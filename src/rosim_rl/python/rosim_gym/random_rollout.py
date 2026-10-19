"""Random-policy rollout smoke test for the rosim RL skeleton.

Imports the sample workpiece (or one given as argv[1]), picks random edges and
requests follows — no torch needed. Exercises exactly the interface surface a
trained policy will use.

  source ~/rosim_ws/install/setup.bash
  python3 random_rollout.py [path/to/workpiece.step]
"""

import os
import random
import sys

import rclpy
from rclpy.node import Node

from rosim_gym.env import RosimEnv


def main():
    step = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/rosim_ws/src/rosim_description/meshes/workpiece_sample.step")
    if not os.path.exists(step):
        print(f"workpiece not found: {step}")
        return 1

    rclpy.init()
    node = Node("rosim_random_rollout")
    env = RosimEnv(node)

    ok, model_id, edge_count = env.import_workpiece(step)
    if not ok:
        print("import failed")
        return 1
    print(f"imported {model_id} with {edge_count} edges")

    edge = random.randrange(edge_count)
    print(f"random edge: {edge}")
    success, fraction = env.follow_edge(model_id, edge)
    print(f"follow_edge -> success={success} fraction={fraction:.2f}")

    obs = env.observation()
    print(f"observation dim={len(obs)}, joints={obs[:6].round(3)}")

    rclpy.shutdown()
    return 0 if success else 2


if __name__ == "__main__":
    sys.exit(main())
