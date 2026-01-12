import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import yaml
from pathlib import Path

from python_motion_planning.utils import Grid, SearchFactory

class LPAStarPlanner(Node):
    def __init__(self):
        super().__init__('lpastar_planner')

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/lpastar_path', 10)

        self.map = None
        self.resolution = None
        self.origin = None
        self.env = None
        self.robot_pose = None

    # ================= MAP ======================
    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data).reshape((h, w))

        # ROS: -1 unknown, 0 free, 100 occupied
        map_bin = (data > 50).astype(np.uint8)

        self.env = Grid(w, h)
        obstacles = {(x, h - 1 - y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
        self.env.update(obstacles)

        self.map = map_bin
        self.get_logger().info("Map loaded")

    # ================= ODOM ======================
    def odom_callback(self, msg):
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # ================= GOAL ======================
    def goal_callback(self, msg):
        if self.map is None or self.robot_pose is None:
            self.get_logger().warn("Waiting for map or odom...")
            return

        goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.plan(self.robot_pose, goal)

    # ================= PLANNING ======================
    def world_to_map(self, x, y):
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return (mx, my)

    def map_to_world(self, x, y):
        wx = x * self.resolution + self.origin[0]
        wy = y * self.resolution + self.origin[1]
        return (wx, wy)

    def plan(self, start_w, goal_w):
        start = self.world_to_map(*start_w)
        goal = self.world_to_map(*goal_w)

        self.get_logger().info(f"Planning {start} -> {goal}")

        planner = SearchFactory()("lpa_star", start=start, goal=goal, env=self.env)
        planner.run()
        cost, path, _ = planner.plan()

        ros_path = Path()
        ros_path.header.frame_id = "map"

        for x,y in reversed(path):
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = self.map_to_world(x,y)
            ros_path.poses.append(p)

        self.path_pub.publish(ros_path)
        self.get_logger().info("Path published")

def main():
    rclpy.init()
    node = LPAStarPlanner()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

