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



#TODO: Import the message types that you need for your publishers and subscribers here:
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BraitenbergController(Node):
    def __init__(self):
        #TODO: Initialze a ros node
        super().__init__('BraitenbergController')

        #TODO: Create a subscriber to the /scan topic using as a callback function the already existing function inside the class called clbk_laser
        # self.scan_sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)

        #TODO: Create a publisher to the /cmd_vel topic
        self.cmdvel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #default values for the lidar variables as a placeholder until the actual sensor values are recieved through from the ros topic
        # self.lidar_left_front = 100
        # self.lidar_right_front = 100

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    #Callback function for the Turtlebots Lidar topic ("/scan")
    # def clbk_laser(self, msg):
    #     self.lidar_left_front = msg.ranges[12]
    #     self.lidar_right_front = msg.ranges[348]

    def timer_callback(self):
        #Creates a message from type Twist
        vel_msg = Twist()
        #Defines the speed at which the robot will move forward (value between 0 and 1)
        vel_msg.linear.x = 0.0
        #Defines the speed at which the robot will turn around its own axis (value between -1 and 1)
        vel_msg.angular.z = 0.0

        # TODO: Set vel_msg.linear.x and vel_msg.angular.z depending on the values from self.lidar_left_front and self.lidar_right_front
        # lidar_threshold = 1.5
        # vel_msg.angular.z = 0.0
        # if self.lidar_left_front < lidar_threshold and self.lidar_right_front < lidar_threshold:
        #     vel_msg.linear.x = 0.0
        #     vel_msg.angular.z = 0.0
        # else:
        #     if self.lidar_left_front < lidar_threshold:
        #         vel_msg.angular.z -= 0.5
        #     if self.lidar_right_front < lidar_threshold:
        #         vel_msg.angular.z += 0.5

        #TODO: Publish vel_msg
        self.cmdvel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = BraitenbergController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()