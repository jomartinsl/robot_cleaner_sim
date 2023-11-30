import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        joint_traj_msg = JointTrajectory()
        joint_traj_msg.header = Header()
        joint_traj_msg.header.stamp = self.get_clock().now().to_msg()
        joint_traj_msg.joint_names = ['base_arm_joint', 'link_1_joint', 'link_2_joint', 'link_3_joint'] # the same as in the Matlab script
        point = JointTrajectoryPoint()
        point.positions = [0.707, 0.0, 0.707, 0.0]
        # point.time_from_start = rclpy.duration.Duration(seconds=1.0)
        joint_traj_msg.points.append(point)

        self.publisher_.publish(joint_traj_msg)
        self.get_logger().info('Publishing')
    


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()