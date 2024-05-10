import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist

class PoseToCmdVel(Node):
    def __init__(self):
        super().__init__('pose_to_cmdvel')
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID init
        self.prev_error = 0.0
        self.integral = 0.0
        self.kp = 0.2
        self.ki = 0.001
        self.kd = 0.0000001

    def pose_callback(self, msg):
        if len(msg.poses) > 0:

            # 포즈 정보를 개별 변수에 저장
            pose_x = msg.poses[0].position.x
            # pose_y = msg.poses[0].position.y
            pose_z = msg.poses[0].position.z
            # pose_qx = msg.poses[0].orientation.x
            # pose_qy = msg.poses[0].orientation.y
            # pose_qz = msg.poses[0].orientation.z
            # pose_qw = msg.poses[0].orientation.w

            error = 0.0 - pose_x
            self.integral += error
            derivative = error - self.prev_error

            angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative

            # 제어값을 제한하여 너무 큰 값이 발생하지 않도록 함
            max_angular_z = 0.8
            if angular_z > max_angular_z:
                angular_z = max_angular_z
            elif angular_z < -max_angular_z:
                angular_z = -max_angular_z
            

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -0.2
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = angular_z


            if(msg.poses[0].position.z < 0.4):
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                
            self.publisher.publish(cmd_vel_msg)
            self.get_logger().info('z: %f' % angular_z)
            
            self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = PoseToCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
