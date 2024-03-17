import rclpy
from src.robot_simul import RobotSimul

def sysCall_init():
    sim = require('sim')

    left_wheel_handle = sim.getObject('./Left_wheel_Joint')
    right_wheel_handle = sim.getObject('./Right_wheel_Joint')

    robot_handle = sim.getObject('.')

    print(sim.getObjectPose(robot_handle))

    rclpy.init()

    self.robot = RobotSimul(robot_handle, left_wheel_handle, right_wheel_handle)

def sysCall_sensing():
    self.robot.publish_pose()
    rclpy.spin_once(self.robot, timeout_sec=0)

def sysCall_cleanup():
    self.robot.destroy_node()
    rclpy.shutdown()