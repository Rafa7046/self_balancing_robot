import rclpy
from src.robot_simul import RobotSimul
import matplotlib.pyplot as plt

def sysCall_init():
    sim = require('sim')

    left_wheel_handle = sim.getObject('./Left_wheel_Joint')
    right_wheel_handle = sim.getObject('./Right_wheel_Joint')

    robot_handle = sim.getObject('.')

    print(sim.getObjectPose(robot_handle))

    rclpy.init()

    self.robot = RobotSimul(sim, robot_handle, left_wheel_handle, right_wheel_handle)

def sysCall_sensing():
    self.robot.publish_pose()
    rclpy.spin_once(self.robot, timeout_sec=0)

def sysCall_cleanup():
    error = self.robot.erros

    # Extract X and Y values from the error list of tuples
    X = [x for x, _ in error]
    Y = [y for _, y in error]

    self.robot.destroy_node()
    rclpy.shutdown()
    # Plot the X and Y values
    plt.plot(X, Y)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Error Plot')
    plt.show()