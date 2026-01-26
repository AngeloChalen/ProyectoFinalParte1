import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
import math, time

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.path, self.curr_pose, self.start_pose = [], None, None
        self.is_moving = False
        self.start_time = 0.0
        self.path_sub = self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Controlador de Precisión (Vel: 0.25) activo.")

    def path_cb(self, msg):
        if msg.poses:
            self.path, self.start_pose = msg.poses, self.curr_pose
            self.start_time, self.is_moving = time.time(), True

    def odom_cb(self, msg): self.curr_pose = msg.pose.pose

    def control_loop(self):
        if not self.path or not self.curr_pose or not self.is_moving: return
        target = self.path[-1].pose.position; robot = self.curr_pose.position
        
        # Métrica exacta desde el punto de inicio (2D Pose Estimate)
        d_rec = math.sqrt((robot.x - self.start_pose.position.x)**2 + (robot.y - self.start_pose.position.y)**2)
        d_meta = math.sqrt((target.x - robot.x)**2 + (target.y - robot.y)**2)
        
        print(f"⏱️ {time.time()-self.start_time:.1f}s | 📏 Recorrido: {d_rec:.2f} m | 🏁 A meta: {d_meta:.2f} m", end='\r')

        if d_meta < 0.20: # Tolerancia aumentada para frenado suave
            self.is_moving = False; self.vel_pub.publish(Twist())
            print(f"\n🏁 META ALCANZADA: Distancia Total {d_rec:.2f} m")
            return

        yaw = math.atan2(2*(self.curr_pose.orientation.w*self.curr_pose.orientation.z), 1 - 2*(self.curr_pose.orientation.z**2))
        err = math.atan2(target.y - robot.y, target.x - robot.x) - yaw
        cmd = Twist(); cmd.linear.x = 0.25; cmd.angular.z = math.atan2(math.sin(err), math.cos(err)) * 1.5
        self.vel_pub.publish(cmd)

def main(args=None): rclpy.init(args=args); n = PurePursuitController(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()