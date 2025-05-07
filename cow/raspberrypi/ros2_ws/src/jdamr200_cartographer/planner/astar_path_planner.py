#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import yaml
import os
import math
from queue import PriorityQueue
from ament_index_python.packages import get_package_share_directory
from scipy.interpolate import splprep, splev

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_path_planner')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.map_image, self.resolution, self.origin = self.load_map()

        h, w = self.map_image.shape
        self.start = (int(w / 2), int(h / 2) + 20)
        self.goal = None

        self.get_logger().info(f"A* Planner initialized. Start: {self.start}. Waiting for /goal_pose...")

    def load_map(self):
        map_dir = os.path.join(
            get_package_share_directory('jdamr200_cartographer'),
            'maps'
        )
        with open(os.path.join(map_dir, 'map.yaml'), 'r') as f:
            map_metadata = yaml.safe_load(f)

        image = cv2.imread(os.path.join(map_dir, map_metadata['image']), cv2.IMREAD_GRAYSCALE)

        # 장애물 확장 (좁은 길 막기, 멀리까지 열어줌)
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(255 - image, kernel, iterations=2)
        image = 255 - dilated

        resolution = map_metadata['resolution']
        origin = map_metadata['origin']
        return image, resolution, origin

    def goal_callback(self, msg):
        x = int((msg.pose.position.x - self.origin[0]) / self.resolution)
        y = int(self.map_image.shape[0] - (msg.pose.position.y - self.origin[1]) / self.resolution)
        self.goal = (x, y)

        self.get_logger().info(f"Received new goal: {self.goal}")
        self.get_logger().info(f"Map value at goal: {self.map_image[self.goal[1], self.goal[0]]}")
        self.get_logger().info(f"Map value at start: {self.map_image[self.start[1], self.start[0]]}")

        self.plan_and_publish()

    def plan_and_publish(self):
        if self.goal is None:
            self.get_logger().warn("Goal not set. Ignoring plan request.")
            return

        raw_path = self.a_star(self.map_image, self.start, self.goal)

        if raw_path:
            path = self.smooth_path(raw_path, smoothing_factor=3.0, num_points=len(raw_path)*2)

            ros_path = Path()
            ros_path.header.frame_id = "map"
            ros_path.header.stamp = self.get_clock().now().to_msg()

            for (x, y) in path:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x * self.resolution + self.origin[0]
                pose.pose.position.y = (self.map_image.shape[0] - y) * self.resolution + self.origin[1]
                pose.pose.orientation.w = 1.0
                ros_path.poses.append(pose)

            self.publisher_.publish(ros_path)
            self.get_logger().info(f'Published smoothed path with {len(ros_path.poses)} poses.')
        else:
            self.get_logger().warn("⚠️ 경로를 찾을 수 없습니다.")

    def is_free(self, cell_value):
        return cell_value >= 230  # 회색(불확실한 영역)도 일부 허용

    def a_star(self, grid, start, goal):
        h, w = grid.shape
        visited = np.zeros_like(grid, dtype=bool)
        came_from = {}
        cost = {start: 0}
        queue = PriorityQueue()
        queue.put((0, start))

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])  # 유클리드 거리

        directions = [
            (-1,0),(1,0),(0,-1),(0,1),
            (-1,-1),(-1,1),(1,-1),(1,1),
            (-2, 0), (2, 0), (0, -2), (0, 2),
            (-2, -2), (2, 2), (-2, 2), (2, -2)
        ]

        while not queue.empty():
            _, current = queue.get()
            if current == goal:
                break

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if (0 <= neighbor[0] < w) and (0 <= neighbor[1] < h):
                    if visited[neighbor[1], neighbor[0]]:
                        continue
                    if not self.is_free(grid[neighbor[1], neighbor[0]]):
                        continue

                    move_cost = math.hypot(dx, dy)
                    new_cost = cost[current] + move_cost

                    if neighbor not in cost or new_cost < cost[neighbor]:
                        cost[neighbor] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        queue.put((priority, neighbor))
                        came_from[neighbor] = current

        # 경로 역추적
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []
        path.append(start)
        path.reverse()
        return path

    def smooth_path(self, path, smoothing_factor=3.0, num_points=100):
        if len(path) < 4:
            return path

        x = [p[0] for p in path]
        y = [p[1] for p in path]

        tck, u = splprep([x, y], s=smoothing_factor)
        unew = np.linspace(0, 1, num_points)
        out = splev(unew, tck)

        return list(zip(out[0], out[1]))

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
