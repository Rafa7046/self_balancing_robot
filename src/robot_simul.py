from src.pidcontrol import PID_Controller
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from rclpy.node import Node
import numpy as np
import math

class RobotSimul(Node):
    def __init__(self, sim, robot_handle, left_wheel_handle, right_wheel_handle):
        super().__init__('robot_simul')
        self.sim = sim
        self.pid = PID_Controller()
        self.robot_handle = robot_handle
        self.left_wheel_handle = left_wheel_handle
        self.right_wheel_handle = right_wheel_handle

        self.lw_sub = self.create_subscription(Float64, 'left_wheel/cmd_vel', self.callback_lw, 10)
        self.rw_sub = self.create_subscription(Float64, 'right_wheel/cmd_vel', self.callback_rw, 10)

        # self.physics_sub = self.create_subscription(bool, 'physics', self.callback_physics, 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

    def callback_physics(self, msg : Float64):
        self.sim.simxSynchronousTrigger()

    def callback_lw(self, msg : Float64):
        self.sim.setJointTargetVelocity(self.left_wheel_handle, msg.data)

    def callback_rw(self, msg : Float64):
        self.sim.setJointTargetVelocity(self.right_wheel_handle, msg.data)
    
    def publish_pose(self):
        pose = self.sim.getObjectPose(self.robot_handle)
        
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