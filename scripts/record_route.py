#!/usr/bin/env python3
"""Record a route while driving with teleop_keyboard.

Samples map->base_footprint from TF and stores a waypoint every MIN_DIST
metres or MIN_TURN degrees. Ctrl+C (SIGINT) saves the route to YAML.

Usage: python3 record_route.py [output.yaml]
"""
import math
import os
import signal
import sys

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

MIN_DIST = 0.7                 # m between waypoints (>= RemovePassedGoals radius)
MIN_TURN = math.radians(45)    # rad of heading change that also triggers a waypoint


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RouteRecorder(Node):
    def __init__(self, out_path):
        super().__init__('route_recorder')
        self.out_path = out_path
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points = []
        self.last = None
        self.create_timer(0.2, self.sample)
        self.get_logger().info(f'Recording to {out_path} (every {MIN_DIST} m / '
                               f'{math.degrees(MIN_TURN):.0f} deg). Ctrl+C to save.')

    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_from_quat(t.transform.rotation))

    def add(self, pose):
        self.points.append(pose)
        self.last = pose
        self.get_logger().info(f'#{len(self.points)}: x={pose[0]:.2f} y={pose[1]:.2f} '
                               f'yaw={math.degrees(pose[2]):.0f}')

    def sample(self):
        pose = self.current_pose()
        if pose is None:
            self.get_logger().warn('No map->base_footprint TF yet (initial pose set?)',
                                   throttle_duration_sec=5.0)
            return
        if self.last is None:
            self.add(pose)
            return
        dist = math.hypot(pose[0] - self.last[0], pose[1] - self.last[1])
        turn = abs(math.atan2(math.sin(pose[2] - self.last[2]), math.cos(pose[2] - self.last[2])))
        if dist >= MIN_DIST or turn >= MIN_TURN:
            self.add(pose)

    def save(self):
        pose = self.current_pose()
        if pose is not None and (self.last is None or
                                 math.hypot(pose[0] - self.last[0], pose[1] - self.last[1]) > 0.05 or
                                 abs(pose[2] - self.last[2]) > math.radians(5)):
            self.add(pose)
        data = {'frame_id': 'map',
                'waypoints': [{'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}
                              for x, y, yaw in self.points]}
        with open(self.out_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        print(f'Saved {len(self.points)} waypoints to {self.out_path}', flush=True)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(
        os.path.abspath(__file__)), 'route.yaml')
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = RouteRecorder(out)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
