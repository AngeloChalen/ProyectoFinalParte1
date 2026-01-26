import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import numpy as np
import heapq
import csv
import os
import datetime

class LpaStarFinder:
    def __init__(self):
        self.grid = None
        self.width, self.height = 0, 0
        self.g, self.rhs, self.U = {}, {}, []
        self.start, self.goal = None, None
        # INFLACIÓN AUMENTADA A 9 PARA EVITAR CHOQUES
        self.inflation_cells = 5

    def update_grid(self, grid_data, width, height):
        self.grid, self.width, self.height = grid_data, width, height

    def is_safe(self, x, y):
        if self.grid is None or not (0 <= x < self.width and 0 <= y < self.height): return False
        r = self.inflation_cells
        # Verificación de colisión optimizada
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.grid[ny * self.width + nx] >= 50: return False
                else: return False
        return True

    def get_neighbors(self, node):
        (x, y) = node
        return [c for c in [(x+1,y),(x-1,y),(x,y+1),(x,y-1),(x+1,y+1),(x+1,y-1),(x-1,y+1),(x-1,y-1)] 
                if self.is_safe(c[0], c[1])]

    def calculate_key(self, s):
        m = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        dist = np.sqrt((self.goal[0]-s[0])**2 + (self.goal[1]-s[1])**2)
        return (m + dist, m)

    def update_vertex(self, u):
        if u != self.start:
            neighbors = self.get_neighbors(u)
            if neighbors:
                self.rhs[u] = min([self.g.get(s, float('inf')) + (1.414 if s[0]!=u[0] and s[1]!=u[1] else 1.0) 
                                   for s in neighbors])
            else:
                self.rhs[u] = float('inf')
        self.U = [i for i in self.U if i[1] != u]
        heapq.heapify(self.U)
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def find_path(self, start, goal):
        self.start, self.goal = start, goal
        self.U, self.g, self.rhs = [], {}, {}
        self.rhs[start], self.g[start] = 0, float('inf')
        heapq.heappush(self.U, (self.calculate_key(start), start))
        
        while self.U and (self.U[0][0] < self.calculate_key(goal) or self.rhs.get(goal) != self.g.get(goal)):
            u = heapq.heappop(self.U)[1]
            if self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u): self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u)
                for s in self.get_neighbors(u): self.update_vertex(s)
        
        if self.g.get(goal, float('inf')) == float('inf'): return None
        path, curr = [goal], goal
        while curr != start:
            neighbors = self.get_neighbors(curr)
            curr = min(neighbors, key=lambda n: self.g.get(n, float('inf')))
            path.append(curr)
        return path[::-1]

class LpaPlannerNode(Node):
    def __init__(self):
        super().__init__('lpa_planner_node')
        self.planner = LpaStarFinder()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_cb, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.tf_br = TransformBroadcaster(self)
        self.map_info, self.curr_pos = None, (0.0, 0.0)
        self.timer = self.create_timer(0.03, self.timer_cb)
        self.get_logger().info('✅ LPA* con Alta Inflación y CSV iniciado.')

    def timer_cb(self):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = self.get_clock().now().to_msg(), 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.curr_pos[0], self.curr_pos[1], 0.35
        t.transform.rotation.w = 1.0; self.tf_br.sendTransform(t)
        m = Marker(); m.header.frame_id, m.header.stamp = "map", t.header.stamp
        m.type, m.scale.x, m.scale.y, m.scale.z = Marker.CUBE, 0.4, 0.2, 0.2
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.curr_pos[0], self.curr_pos[1], 0.35
        m.color.r, m.color.a = 1.0, 0.6; self.marker_pub.publish(m)

    def map_cb(self, msg): 
        self.map_info = msg.info
        self.planner.update_grid(msg.data, msg.info.width, msg.info.height)

    def odom_cb(self, msg): self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    def init_cb(self, msg): self.curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_cb(self, msg):
        if not self.map_info: return
        self.get_logger().info('🏁 Calculando ruta segura...')
        s = self.w2g(self.curr_pos[0], self.curr_pos[1])
        g = self.w2g(msg.pose.position.x, msg.pose.position.y)
        path = self.planner.find_path(s, g)
        if path:
            p_msg = Path(); p_msg.header.frame_id, p_msg.header.stamp = 'map', self.get_clock().now().to_msg()
            world_pts = []
            for (mx, my) in path:
                p = PoseStamped(); wx, wy = self.g2w(mx, my)
                p.pose.position.x, p.pose.position.y, p.pose.orientation.w = wx, wy, 1.0
                p_msg.poses.append(p); world_pts.append([wx, wy])
            self.path_pub.publish(p_msg)
            self.save_csv(world_pts)
            self.get_logger().info(f'🚀 Ruta segura generada y CSV guardado.')
        else: self.get_logger().error('❌ Error: Meta en zona de colisión.')

    def save_csv(self, points):
        try:
            folder = os.path.join(os.path.expanduser('~'), 'go2_ws')
            if not os.path.exists(folder): os.makedirs(folder)
            fn = os.path.join(folder, f"evidencia_{datetime.datetime.now().strftime('%H%M%S')}.csv")
            with open(fn, 'w', newline='') as f:
                writer = csv.writer(f); writer.writerow(['X', 'Y']); writer.writerows(points)
            self.get_logger().info(f'💾 CSV: {fn}')
        except Exception as e: self.get_logger().error(f'CSV Error: {e}')

    def w2g(self, wx, wy):
        res = self.map_info.resolution
        return (int((wx - self.map_info.origin.position.x) / res), int((wy - self.map_info.origin.position.y) / res))

    def g2w(self, mx, my):
        res = self.map_info.resolution
        return (mx*res + self.map_info.origin.position.x + res/2, my*res + self.map_info.origin.position.y + res/2)

def main(args=None): rclpy.init(args=args); n = LpaPlannerNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
