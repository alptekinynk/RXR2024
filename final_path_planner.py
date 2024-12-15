#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
import math


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")
        self.global_costmap = None
        self.local_costmap = None

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribe costmap's
        self.create_subscription(OccupancyGrid, '/map', self.occupancy_grid_callback, qos_profile)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, qos_profile)

        # Planing
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

    def occupancy_grid_callback(self, msg):
        self.get_logger().info("Global Costmap alındı!")
        self.global_costmap = msg

    def local_costmap_callback(self, msg):
        self.local_costmap = msg

    def create_plan_cb(self, request, response):
        if self.global_costmap is None:
            self.get_logger().error("Global costmap henüz yüklenmedi!")
            return response

        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()

        # Straight path according to global costmap
        straight_path = self.create_straight_plan(start_pose, goal_pose, time_now)

        # Create new route if you see obstacle
        if self.local_costmap and self.has_obstacles_in_path(straight_path):
            self.get_logger().info("Engel algılandı! Basit engelden kaçma algoritması uygulanıyor.")
            start_pose = self.avoid_obstacle(start_pose)
            straight_path = self.create_straight_plan(start_pose, goal_pose, time_now)

        response.path = straight_path
        return response

    def create_straight_plan(self, start, goal, time_now):
        path = Path()
        path.header.frame_id = goal.header.frame_id
        path.header.stamp = time_now

        interpolated_points = self.interpolate_coordinates(
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

    def has_obstacles_in_path(self, path):
        """
        Rota üzerinde engel olup olmadığını kontrol eder.
        """
        if not self.local_costmap:
            return False

        costmap_array = self.convert_costmap_to_2d(self.local_costmap)
        for pose in path.poses:
            x, y = int(pose.pose.position.x), int(pose.pose.position.y)
            if 0 <= y < len(costmap_array) and 0 <= x < len(costmap_array[0]) and costmap_array[y][x] > 0:
                return True
        return False

    def convert_costmap_to_2d(self, costmap):
        """
        OccupancyGrid'yi 2D diziye çevirir.
        """
        width = costmap.info.width
        height = costmap.info.height
        data = costmap.data
        grid = [data[i * width:(i + 1) * width] for i in range(height)]
        return grid

    def avoid_obstacle(self, start_pose):
        """
        Engelden kaçmak için yön değiştirir ve yeni bir başlangıç pozisyonu döner.
        """
        # Curret positions
        current_x = start_pose.pose.position.x
        current_y = start_pose.pose.position.y

        # Simple avoid 
        new_x = current_x + 0.5 
        new_y = current_y

        # Update position
        start_pose.pose.position.x = new_x
        start_pose.pose.position.y = new_y
        self.get_logger().info(f"Engelden kaçmak için yeni başlangıç pozisyonu: ({new_x}, {new_y})")
        return start_pose

    def interpolate_coordinates(self, start, end, increment=0.1):
        x1, y1 = start
        x2, y2 = end

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx**2 + dy**2)
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
    node = PathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
