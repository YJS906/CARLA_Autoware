#!/usr/bin/env python3
"""Read-only ROS goal cache for save_goal_speed.py; runs in the Autoware container."""
import fcntl
import json
import os
import time
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

VERSION = 1
DEST = Path('/tmp/autoware-goal-speed.json')


def main():
    lock = open('/tmp/autoware-goal-speed.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    rclpy.init()
    node = rclpy.create_node('goal_speed_read_only_cache')
    state = {'version': VERSION, 'pid': os.getpid(), 'goal': None}
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def update(header, pose, source):
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        old = state['goal']
        # Late delivery of a retained route must not replace a newer RViz click.
        if old and stamp < old['stamp_ns']:
            return
        state['goal'] = {
            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'frame': header.frame_id, 'stamp_ns': stamp, 'source': source,
            'received_at': time.time(),
        }

    subscriptions = [
        node.create_subscription(LaneletRoute, '/planning/mission_planning/route',
            lambda m: update(m.header, m.goal_pose, 'route'), latched),
        node.create_subscription(PoseStamped, '/planning/mission_planning/echo_back_goal_pose',
            lambda m: update(m.header, m.pose, 'echo_back_goal_pose'), latched),
        node.create_subscription(PoseStamped, '/planning/mission_planning/goal',
            lambda m: update(m.header, m.pose, '2d_goal_pose'), reliable),
    ]
    last_write = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_write >= 0.2:
                state['alive_at'] = now
                temporary = DEST.with_suffix('.tmp')
                temporary.write_text(json.dumps(state))
                os.replace(temporary, DEST)
                last_write = now
    finally:
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
