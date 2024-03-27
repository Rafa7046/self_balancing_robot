from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from rclpy.node import Node
import numpy as np
import math
import rclpy
from src.pidcontrol import PID_Controller

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.lw_pub = self.create_publisher(Float64, 'left_wheel/cmd_vel', 10)
        self.rw_pub = self.create_publisher(Float64, 'right_wheel/cmd_vel', 10)

        self.pose_sub = self.create_subscription(Pose, 'pose', self.callback_pose, 10)

        self.controller = PID_Controller()

        self.physics = self.create_publisher(bool, 'physics', 10)
        self.physics.publish(True)

    def publish_wheel_velocities(self, msg):
        self.lw_pub.publish(msg)

        self.rw_pub.publish(msg)

    def callback_pose(self, pose : Pose):
        x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        roll  = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)*180/math.pi
        pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)*180/math.pi
        yaw   =  math.asin(2*x*y + 2*z*w)*180/math.pi
        # print(roll)
        # vel = sim.getJointVelocity(self.left_wheel_handle)
        wheels_velocities = self.controller.getCorrection(0.0, roll)
        msg = Float64()
        msg.data = np.clip(wheels_velocities, -26, 26)

        self.publish_wheel_velocities(msg)

if __name__ == '__main__':
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()