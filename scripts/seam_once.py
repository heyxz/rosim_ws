import sys, time, rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosim_interfaces.action import FollowSeams
from rosim_interfaces.msg import SeamSpec

rclpy.init(); n = Node("seam_once")
cli = ActionClient(n, FollowSeams, "/rosim_planning/follow_seams")
if not cli.wait_for_server(15.0):
    print("action server 不可用"); sys.exit(1)

import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument("model_id")
_ap.add_argument("--approach", type=float, default=0.08)
_ap.add_argument("--tilt", type=float, default=0.6)
_args, _ = _ap.parse_known_args()

goal = FollowSeams.Goal()
goal.model_id = _args.model_id
spec = SeamSpec(); spec.edge_indices = [6, 10]
goal.seams = [spec]
goal.time_profile = 1
goal.seam_constant_speed = True
goal.weld_speed = 0.020
goal.seam_dwell = 0.0
goal.approach_distance = _args.approach
goal.max_waypoint_spacing = 0.005
goal.tangent_tilt = 0.6
goal.corner_max_deviation = 0.0005
goal.corner_threshold_deg = 8.0
goal.corner_orient_window = 0.02

def fb(msg):
    f = msg.feedback
    if f.note:
        print("NOTE:", f.note, flush=True)

send = cli.send_goal_async(goal, feedback_callback=fb)
rclpy.spin_until_future_complete(n, send, timeout_sec=20)
gh = send.result()
if gh is None or not gh.accepted:
    print("goal 被拒"); sys.exit(1)
res = gh.get_result_async()
rclpy.spin_until_future_complete(n, res, timeout_sec=180)
r = res.result().result
msgs = getattr(r, "failed_msg", None)
detail = (" | " + "; ".join(msgs)) if msgs else ""
print(f"result: success={r.success} executed={r.executed} failed={r.failed}{detail}")
