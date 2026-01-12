import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
import heapq
import csv
import os
import datetime

# --- CLASE LPA* REAL (Con g, rhs y keys) ---
class LpaStarFinder:
    def __init__(self):
        self.grid = None
        self.width = 0
        self.height = 0
        self.g = {}
        self.rhs = {}
        self.U = [] # Priority Queue
        self.start = None
        self.goal = None

    def update_grid(self, grid_data, width, height):
        self.grid = grid_data
        self.width = width
        self.height = height

    def heuristic(self, a, b):
        # Distancia Euclidiana
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def calculate_key(self, s):
        # Clave k = [k1, k2] para LPA*
        # k1 = min(g(s), rhs(s)) + h(s, goal)
        # k2 = min(g(s), rhs(s))
        min_val = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        return (min_val + self.heuristic(s, self.goal), min_val)

    def get_neighbors(self, node):
        (x, y) = node
        candidates = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), 
                      (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)]
        result = []
        for nx, ny in candidates:
            if 0 <= nx < self.width and 0 <= ny < self.height:
                index = ny * self.width + nx
                # Consideramos libre si < 50
                if self.grid[index] < 50 and self.grid[index] >= 0:
                    result.append((nx, ny))
        return result

    def update_vertex(self, u):
        if u != self.start:
            # rhs(u) = min(g(s') + cost(s', u)) para todo vecino s'
            min_rhs = float('inf')
            for sprime in self.get_neighbors(u):
                # Costo de movimiento (1.0 o 1.414)
                cost = 1.414 if (sprime[0]!=u[0] and sprime[1]!=u[1]) else 1.0
                # Nota: En LPA* puro, miramos los predecesores. En grid no dirigido, vecinos = predecesores.
                curr_g = self.g.get(sprime, float('inf'))
                if curr_g + cost < min_rhs:
                    min_rhs = curr_g + cost
            self.rhs[u] = min_rhs

        # Gestionar la cola de prioridad U
        # Primero eliminamos u si ya está (forma simplificada reconstruyendo heap)
        self.U = [item for item in self.U if item[1] != u]
        heapq.heapify(self.U)

        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        while self.U:
            u_key, u = self.U[0] # Top key
            
            # Condición de parada: Si la llave del goal es mayor que la top key
            # O si rhs(goal) != g(goal), seguimos procesando.
            # Simplificación para Grid estático: Paramos si llegamos al goal y es consistente
            if self.calculate_key(self.goal) <= u_key and \
               self.rhs.get(self.goal, float('inf')) == self.g.get(self.goal, float('inf')):
                break
            
            heapq.heappop(self.U)
            
            if self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                self.update_vertex(u) # u es vecino de sí mismo en la lógica de actualización
                for s in self.get_neighbors(u):
                    self.update_vertex(s)

    def find_path(self, start, goal):
        # Inicialización de LPA* para una nueva búsqueda
        self.start = start
        self.goal = goal
        self.U = []
        self.g = {}
        self.rhs = {}
        
        self.rhs[start] = 0
        self.g[start] = float('inf')
        self.rhs[goal] = float('inf')
        self.g[goal] = float('inf')

        heapq.heappush(self.U, (self.calculate_key(start), start))
        
        # Ejecutar el núcleo del algoritmo
        self.compute_shortest_path()

        # Reconstruir el camino (Backtracking desde el Goal)
        if self.g.get(goal, float('inf')) == float('inf'):
            return None # No hay camino

        path = [goal]
        current = goal
        while current != start:
            min_dist = float('inf')
            best_next = None
            for neighbor in self.get_neighbors(current):
                cost = 1.414 if (neighbor[0]!=current[0] and neighbor[1]!=current[1]) else 1.0
                dist = self.g.get(neighbor, float('inf')) + cost
                if dist < min_dist:
                    min_dist = dist
                    best_next = neighbor
            
            if best_next and min_dist < float('inf'):
                path.append(best_next)
                current = best_next
            else:
                break # Camino roto
        
        return path[::-1]

# --- NODO ROS 2 (IGUAL QUE ANTES, PERO LLAMA A LPA*) ---
class LpaPlannerNode(Node):
    def __init__(self):
        super().__init__('lpa_planner_node')
        # Usamos la nueva clase LPA*
        self.planner = LpaStarFinder()
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) 
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.map_info = None
        self.current_pose = (0.0, 0.0) 
        self.csv_folder = '/home/knezevich/go2_ws/'
        
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info('✅ Nodo Listo: Algoritmo LPA* implementado.')

    def timer_callback(self):
        self.broadcast_tf(self.current_pose)
        self.publish_robot_marker(self.current_pose)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.planner.update_grid(msg.data, msg.info.width, msg.info.height)
        self.get_logger().info('🗺️ Mapa Recibido.')

    def odom_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def initial_pose_callback(self, msg):
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        self.current_pose = (x, y)
        print(f"\n📍 ¡CLICK DETECTADO! Robot movido a: ({x:.2f}, {y:.2f})")

    def goal_callback(self, msg):
        if not self.map_info: return
        print(f"🏁 Calculando ruta con LPA*...")
        goal_x = msg.pose.position.x; goal_y = msg.pose.position.y
        start_grid = self.world_to_grid(self.current_pose[0], self.current_pose[1])
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        # Llamada al planificador LPA*
        path_grid = self.planner.find_path(start_grid, goal_grid)
        
        if path_grid:
            world_path = self.publish_path(path_grid)
            self.save_csv_unique(world_path)
            print(f"🚀 ¡ÉXITO! Ruta generada.")
        else: self.get_logger().error('❌ No se encontró camino o meta inalcanzable.')

    def broadcast_tf(self, position):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'; t.child_frame_id = 'base_link' 
        t.transform.translation.x = position[0]; t.transform.translation.y = position[1]; t.transform.translation.z = 0.35 
        t.transform.rotation.w = 1.0 
        self.tf_broadcaster.sendTransform(t)

    def publish_robot_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "map"; marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_shape"; marker.id = 0; marker.type = Marker.CUBE; marker.action = Marker.ADD
        marker.pose.position.x = position[0]; marker.pose.position.y = position[1]; marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.0 
        self.marker_pub.publish(marker)

    def save_csv_unique(self, points):
        try:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"ruta_{timestamp}.csv"
            full_path = os.path.join(self.csv_folder, filename)
            with open(full_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['X', 'Y']); writer.writerows(points)
            print(f"💾 CSV: {filename}")
        except Exception: pass

    def world_to_grid(self, wx, wy):
        origin_x = self.map_info.origin.position.x; origin_y = self.map_info.origin.position.y; res = self.map_info.resolution
        mx = int((wx - origin_x) / res); my = int((wy - origin_y) / res)
        mx = max(0, min(mx, self.map_info.width - 1)); my = max(0, min(my, self.map_info.height - 1))
        return (mx, my)

    def grid_to_world(self, mx, my):
        origin_x = self.map_info.origin.position.x; origin_y = self.map_info.origin.position.y; res = self.map_info.resolution
        wx = (mx * res) + origin_x + (res / 2); wy = (my * res) + origin_y + (res / 2)
        return (wx, wy)

    def publish_path(self, path_grid):
        path_msg = Path()
        path_msg.header = Header(); path_msg.header.stamp = self.get_clock().now().to_msg(); path_msg.header.frame_id = 'map'
        world_points = []
        for (mx, my) in path_grid:
            pose = PoseStamped(); pose.header = path_msg.header
            wx, wy = self.grid_to_world(mx, my)
            pose.pose.position.x = wx; pose.pose.position.y = wy; pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose); world_points.append((wx, wy)) 
        self.path_pub.publish(path_msg); return world_points

def main(args=None):
    rclpy.init(args=args); node = LpaPlannerNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
