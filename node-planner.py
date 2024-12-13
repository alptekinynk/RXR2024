#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def create_plan_cb(self, request, response):
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()

        print("----")
        print(f"Starting pose: ({start_pose.pose.position.x}, {start_pose.pose.position.y})")
        print(f"Goal pose: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}))")

        response.path = create_straight_plan(start_pose, goal_pose, time_now)
        return response


def create_straight_plan(start, goal, time_now):
    path = Path()

    
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    
    interpolated_points = interpolate_coordinates(
        (start.pose.position.x, start.pose.position.y),
        (goal.pose.position.x, goal.pose.position.y)
    )

    
    for point in interpolated_points:
        pose = PoseStamped()
        pose.header.frame_id = goal.header.frame_id
        pose.header.stamp = time_now
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        path.poses.append(pose)

    return path


def interpolate_coordinates(start, end, increment=0.1):
    x1, y1 = start
    x2, y2 = end

    dx = x2 - x1
    dy = y2 - y1
    distance = (dx ** 2 + dy ** 2) ** 0.5

    num_steps = int(distance / increment)
    points = []

    for i in range(num_steps + 1):
        t = i / num_steps
        x = x1 + t * dx
        y = y1 + t * dy
        points.append((x, y))

    return points


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()

    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
