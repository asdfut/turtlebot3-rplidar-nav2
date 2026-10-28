#!/usr/bin/env python3
"""Drive a recorded route with Nav2.

Default mode passes the whole route to NavigateThroughPoses (continuous motion).
--stop uses FollowWaypoints instead (stops at every waypoint).

Usage: python3 follow_route.py [route.yaml] [--stop] [--loops N]
"""
import argparse
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# turtlebot3_navigation2's burger.yaml sets default_nav_through_poses_bt_xml to the
# navigate_to_pose tree, which ignores the pose list, so pass the right tree per goal.
THROUGH_POSES_BT = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
                                'navigate_through_poses_w_replanning_and_recovery.xml')


def make_pose(nav, wp, frame_id):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(wp['x'])
    p.pose.position.y = float(wp['y'])
    p.pose.orientation.z = math.sin(wp['yaw'] / 2)
    p.pose.orientation.w = math.cos(wp['yaw'] / 2)
    return p


def main():
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'route.yaml')
    ap = argparse.ArgumentParser()
    ap.add_argument('route', nargs='?', default=default)
    ap.add_argument('--stop', action='store_true', help='stop at each waypoint (FollowWaypoints)')
    ap.add_argument('--loops', type=int, default=1)
    args = ap.parse_args()

    with open(args.route) as f:
        data = yaml.safe_load(f)
    wps = data['waypoints']
    frame_id = data.get('frame_id', 'map')

    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active(localizer='amcl')

    def wait(label):
        i = 0
        while not nav.isTaskComplete():
            i += 1
            fb = nav.getFeedback()
            if fb and i % 10 == 0:
                if hasattr(fb, 'current_waypoint'):
                    print(f'  {label}: waypoint {fb.current_waypoint + 1}', flush=True)
                else:
                    print(f'  {label}: remaining {fb.distance_remaining:.2f} m', flush=True)
            rclpy.spin_once(nav, timeout_sec=0.1)
        res = nav.getResult()
        print(f'  {label}: {res.name}', flush=True)
        return res

    result = None
    for loop in range(args.loops):
        poses = [make_pose(nav, wp, frame_id) for wp in wps]
        print(f'Loop {loop + 1}/{args.loops}: {len(poses)} waypoints', flush=True)

        # Drive to the route start first: the controller only checks the end of the
        # path, so a route ending where the robot stands would "succeed" instantly.
        nav.goToPose(poses[0])
        result = wait('to start')
        if result == TaskResult.SUCCEEDED and len(poses) > 1:
            if args.stop:
                nav.followWaypoints(poses[1:])
            else:
                nav.goThroughPoses(poses[1:], behavior_tree=THROUGH_POSES_BT)
            result = wait('route')

        print(f'Loop {loop + 1} result: {result.name}', flush=True)
        if result != TaskResult.SUCCEEDED:
            break

    nav.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result == TaskResult.SUCCEEDED else 1)


if __name__ == '__main__':
    main()
