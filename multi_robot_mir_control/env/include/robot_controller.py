# mir_gym_env/include/robot_controller.py

import rospy
from geometry_msgs.msg import Twist

class RobotController:
    """
    Đóng gói việc gửi lệnh điều khiển tới MỘT robot.
    """
    def __init__(self, robot_name):
        self.robot_name = robot_name
        cmd_vel_topic = f'/{self.robot_name}/mobile_base_controller/cmd_vel'
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def send_command(self, linear_v, angular_w):
        """Gửi lệnh vận tốc tuyến tính và vận tốc góc."""
        twist = Twist()
        twist.linear.x = float(linear_v)
        twist.angular.z = float(angular_w)
        self.cmd_pub.publish(twist)

    def stop(self):
        """Dừng robot."""
        self.send_command(0.0, 0.0)