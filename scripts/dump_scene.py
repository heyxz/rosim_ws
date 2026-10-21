#!/usr/bin/env python3
"""把 /planning_scene 里的碰撞体导出成 JSON（训练侧要用真实几何）。

    python3 scripts/dump_scene.py --out .scratch/scene.json

输出结构：
    {"objects": [{"id": ..., "frame": ..., "pose": [x,y,z,qx,qy,qz,qw],
                  "type": "mesh"|"box",
                  "vertices": [[x,y,z], ...], "triangles": [[i,j,k], ...],
                  "size": [x,y,z]}, ...]}

mesh 是 shape_msgs/Mesh（顶点 + 三角索引），box 直接用尺寸。
"""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from moveit_msgs.msg import PlanningScene


class SceneDumper(Node):
    def __init__(self):
        super().__init__("scene_dumper")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.msg = None
        self.create_subscription(PlanningScene, "/planning_scene", self._cb, qos)

    def _cb(self, msg: PlanningScene):
        self.msg = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=".scratch/scene.json")
    ap.add_argument("--timeout", type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = SceneDumper()
    import time
    t0 = time.time()
    while node.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.msg is None:
        print("超时：没有收到 /planning_scene（栈起来了吗？工件导入了吗？）")
        return 1

    out = {"objects": []}
    for obj in node.msg.world.collision_objects:
        rec = {"id": obj.id, "frame": obj.header.frame_id,
               "pose": [float(obj.pose.position.x), float(obj.pose.position.y),
                        float(obj.pose.position.z), float(obj.pose.orientation.x),
                        float(obj.pose.orientation.y), float(obj.pose.orientation.z),
                        float(obj.pose.orientation.w)]}
        for shape in obj.primitives:
            if shape.type == shape.BOX:
                rec["type"] = "box"
                rec["size"] = [float(x) for x in shape.dimensions]
        for mesh in obj.meshes:
            rec["type"] = "mesh"
            rec["vertices"] = [[float(v.x), float(v.y), float(v.z)] for v in mesh.vertices]
            rec["triangles"] = [[int(t.vertex_indices[0]), int(t.vertex_indices[1]),
                                 int(t.vertex_indices[2])] for t in mesh.triangles]
        if "type" in rec:
            out["objects"].append(rec)

    with open(args.out, "w") as f:
        json.dump(out, f)
    print(f"导出 {len(out['objects'])} 个碰撞体 -> {args.out}")
    for o in out["objects"]:
        n = len(o.get("vertices", []))
        print(f"  {o['id']:20s} {o['type']:5s} 顶点 {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
