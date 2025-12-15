import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time
import math # PI sayısı için

class JumpingRobot(Node):
    def __init__(self):
        super().__init__('jumping_node')

        # --- AYARLAR ---
        self.CROUCH_TIME = 1.5      # Hazırlanma
        self.JUMP_TIME = 0.05       # Patlama
        self.RECOVERY_TIME = 1.0    # Toparlanma
        
        # ==========================================================
        # 180 DERECE THIGH TEST AYARLARI
        # ==========================================================
        # Format: [Hip (Kalça), Thigh (Üst), Calf (Alt)]
        
        # 1. ÇÖMELME (EXTREME LOADING)
        # Thigh (Üst Bacak): 3.14 Radyan (Tam 180 Derece!)
        # Calf (Alt Bacak): -2.1 (Eskisi gibi, kilitli)
        # Bu açı ile bacak tam tur dönmeye veya gövdeye yapışmaya çalışacak.
        self.POSE_CROUCH = [0.0, 0.9, -2.1] 
        
        # 2. ZIPLAMA (FIRLATMA)
        # Burası aynı kalıyor ki fark oluşsun ve zıplasın.
        # 180 dereceden 0 dereceye inmek çok büyük bir momentum yaratır.
        self.POSE_JUMP   = [0.0, -1.3, -0.5] 
        
        # 3. İNİŞ
        self.POSE_LAND   = [0.0, 0.5, -1.2]

        # --- ROS YAPISI ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointTrajectory, 'joint_group_effort_controller/joint_trajectory', 10)
        
        self.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]

        self.state = "IDLE" 
        self.state_start_time = 0
        self.has_jumped = False 
        self.timer = self.create_timer(0.01, self.control_loop)
        self.req_vel = Twist()
        self.get_logger().info("EXTREME ANGLE MODE: Thigh set to 180 Degrees (3.14 rad)!")

    def cmd_vel_callback(self, msg):
        self.req_vel = msg
        if msg.linear.x == 0.0 and self.state == "IDLE":
            self.has_jumped = False

    def publish_pose(self, hip, thigh, calf, duration_sec):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [hip, thigh, calf] * 4
        
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        
        msg.points.append(point)
        self.joint_pub.publish(msg)

    def control_loop(self):
        current_time = time.time()
        is_triggered = abs(self.req_vel.linear.x) > 0.0

        if self.state == "IDLE":
            if is_triggered and not self.has_jumped:
                self.get_logger().info("180 Dereceye Kuruluyor...")
                self.state = "CROUCHING"
                self.state_start_time = current_time
                self.publish_pose(self.POSE_CROUCH[0], self.POSE_CROUCH[1], self.POSE_CROUCH[2], self.CROUCH_TIME)

        elif self.state == "CROUCHING":
            if current_time - self.state_start_time > self.CROUCH_TIME:
                self.get_logger().info("MAX GÜÇ ZIPLA!")
                self.state = "JUMPING"
                self.state_start_time = current_time
                self.publish_pose(self.POSE_JUMP[0], self.POSE_JUMP[1], self.POSE_JUMP[2], self.JUMP_TIME)

        elif self.state == "JUMPING":
            if current_time - self.state_start_time > self.JUMP_TIME + 0.15:
                self.state = "RECOVERING"
                self.state_start_time = current_time
                self.publish_pose(self.POSE_LAND[0], self.POSE_LAND[1], self.POSE_LAND[2], 0.3)

        elif self.state == "RECOVERING":
            if current_time - self.state_start_time > self.RECOVERY_TIME:
                self.state = "IDLE"
                self.has_jumped = True
                self.get_logger().info("Tamamlandı.")

def main(args=None):
    rclpy.init(args=args)
    node = JumpingRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()