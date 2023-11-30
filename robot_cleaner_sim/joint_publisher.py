import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.header = Header()
        joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        joint_traj_msg.joint_names = ['base_arm_joint', 'link_1_joint', 'link_2_joint', 'link_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.707, 0.0, 0.707, 0.0]
        joint_traj_msg.points.append(point)

        self.publisher_.publish(joint_traj_msg)
        self.get_logger().info('Publishing')


class BraitenbergController(Node):
    def __init__(self):
        super().__init__('BraitenbergController')

        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0

        self.cmdvel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    controller = BraitenbergController()

    rclpy.spin(minimal_publisher)
    rclpy.spin(controller)

    minimal_publisher.destroy_node()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()