from src.pidcontrol import PID_Controller
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from rclpy.node import Node
import numpy as np
import math

class RobotSimul(Node):
    def __init__(self, robot_handle, left_wheel_handle, right_wheel_handle):
        super().__init__('robot_simul')
        self.pid = PID_Controller()
        self.robot_handle = robot_handle
        self.left_wheel_handle = left_wheel_handle
        self.right_wheel_handle = right_wheel_handle
        
        self.lw_sub = self.create_subscription(Float64, 'left_wheel/cmd_vel', self.callback_lw, 10)
        self.rw_sub = self.create_subscription(Float64, 'right_wheel/cmd_vel', self.callback_rw, 10)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.callback_pose, 10)

        self.lw_pub = self.create_publisher(Float64, 'left_wheel/cmd_vel', 10)
        self.rw_pub = self.create_publisher(Float64, 'right_wheel/cmd_vel', 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)
        self.publish_wheel_velocities(Float64(data=9.0))

    def callback_lw(self, msg : Float64):
        sim.setJointTargetVelocity(self.left_wheel_handle, msg.data)

    def callback_rw(self, msg : Float64):
        sim.setJointTargetVelocity(self.right_wheel_handle, msg.data)

    def publish_wheel_velocities(self, msg):
        # print(msg.data)
        self.lw_pub.publish(msg)

        self.rw_pub.publish(msg)

    def callback_pose(self, pose : Pose):
        x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        roll  = math.atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z)
        pitch = math.atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z)*180/math.pi
        yaw   =  math.asin(2*x*y + 2*z*w)*180/math.pi
        print(roll)
        # vel = sim.getJointVelocity(self.left_wheel_handle)
        wheels_velocities = self.pid.getCorrection(-0.1, roll)

        msg = Float64()
        msg.data = np.clip(wheels_velocities, -30, 30)

        self.publish_wheel_velocities(msg)
    
    def publish_pose(self):
        # Getting pose relative to the base
        # pose: pose array: [x y z qx qy qz qw]
        pose = sim.getObjectPose(self.robot_handle)
        
        # Creating the JointState structure
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]
        
        # Publishing message
        self.pose_pub.publish(msg)