import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose


class RobotSimul(Node):
    def __init__(self, robot_handle, left_wheel_handle, right_wheel_handle):
        super().__init__('robot_simul')

        self.robot_handle = robot_handle
        self.left_wheel_handle = left_wheel_handle
        self.right_wheel_handle = right_wheel_handle
        
        self.lw_sub = self.create_subscription(Float64, 'left_wheel/cmd_vel', self.callback_lw, 10)
        self.rw_sub = self.create_subscription(Float64, 'right_wheel/cmd_vel', self.callback_rw, 10)

        self.lw_pub = self.create_publisher(Float64, 'left_wheel/vel', 10)
        self.rw_pub = self.create_publisher(Float64, 'right_wheel/vel', 10)

        self.pose_pub = self.create_publisher(Pose, 'pose', 10)

    def callback_lw(self, msg : Float64):
        sim.setJointTargetVelocity(self.left_wheel_handle, msg.data)

    def callback_rw(self, msg : Float64):
        sim.setJointTargetVelocity(self.right_wheel_handle, msg.data)
    
    def publish_wheel_velocities(self):
        msg = Float64()

        msg.data = sim.getJointVelocity(self.left_wheel_handle)

        self.lw_pub.publish(msg)

        msg = Float64()

        msg.data = sim.getJointVelocity(self.right_wheel_handle)

        self.rw_pub.publish(msg)
    
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


def sysCall_init():
    sim = require('sim')

    left_wheel_handle = sim.getObject('./Left_wheel_Joint')
    right_wheel_handle = sim.getObject('./Right_wheel_Joint')

    robot_handle = sim.getObject('.')

    print(sim.getObjectPose(robot_handle))

    rclpy.init()

    self.robot = RobotSimul(robot_handle, left_wheel_handle, right_wheel_handle)

def sysCall_sensing():
    self.robot.publish_wheel_velocities()
    self.robot.publish_pose()
    rclpy.spin_once(self.robot, timeout_sec=0)

def sysCall_cleanup():
    self.robot.destroy_node()
    rclpy.shutdown()