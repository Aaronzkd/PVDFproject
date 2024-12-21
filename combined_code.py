######################################

#!/usr/bin/python3

import rtde_control # real time communication with robot
import rtde_receive
import numpy as np
import transforms3d as tf
from scipy.spatial.transform import Rotation as R
import time
# from geometry_msgs.msg import WrenchStamped
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
# Import the CommandAndDataCollector class
from arduino_data_collector.data_collector import CommandAndDataCollector


class EEPrimitives(Node):

  def __init__(self,exp, threshold_z=15, threshold_xy=13):
    super().__init__('ee_primitives_node')
    self.robot_ip = '192.168.0.216' # fixed

    # Initialize CommandAndDataCollector object
    self.data_collector = CommandAndDataCollector()

    ## set TCP offset
    # self.rtde_c.setTcp([0, 0, 0.0526, 0, 0, 0])
    self.step = -0.005
    self.pose = exp
    ## speed in joint and tool space are different (speed is slower in joint space)
    self.speedJ = 0.5  # m/s
    self.accelJ = 0.3  # m/s^2
    self.speedL = 0.1  # m/s
    self.accelL = 0.2  # m/s^2

    self.speedD = 0.05  # m/s
    self.accelD = 0.05  # m/s^2

    self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
    self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
    
    # Set TCP offset
    self.tcp_offset = [-0.023, 0, 0.25, 0, 0, 0]  # good one, move the EE frame to here
    self.rtde_c.setTcp(self.tcp_offset)
    self.tcp_pose = self.rtde_r.getActualTCPPose()

    # ROS2 subscriber
    self.contact_detected = False
    self.subscription = self.create_subscription(
        Bool,
        'contact_detected',
        self.contact_callback,
        10
    )
    # self.timer=self.create_timer(0.001, self.continuous_move_down)
    
  def contact_callback(self, msg):
        """Callback function for the contact detection topic."""
        self.get_logger().info(f"Contact detected: {msg .data}")
        if msg.data:
            self.contact_detected = True
            self.rtde_c.stopL()  # Stop linear motion immediately
            print("you should stop")
            time.sleep(0.5)
            self.run_data_collection()

  def euler_to_rotvec_pose(self, pose):

    axes, angle = tf.euler.euler2axangle(math.radians(pose[3]), math.radians(pose[4]), math.radians(pose[5]))
    rotvec = axes * angle
    pose[3:] = rotvec

    return pose

  def move_j_to_start(self, pose):
      """ Move to start joint pose """
      print('Moving to initial joint pose: ', pose)
      self.rtde_c.moveJ(pose, self.speedJ, self.accelJ)
      print('At start pose')


  def move_l_to_start(self, pose):
    """ move to start pose for experiments  """
    print('moving to start pose: ', pose)
    self.rtde_c.moveL(pose, self.speedL, self.accelL)
    print('at the pose for experiment!')

  def continuous_move_down(self,exp):
    """Move down incrementally along the z-axis."""

    exp[2] -= 0.027    # Set the limit to protect the grippers

    self.rtde_c.moveL(exp, self.speedD, self.accelD, asynchronous= True)
    # time.sleep(0.1)  # Small delay for smooth movement

  def disconnect(self):
    self.get_logger().info('Disconnecting from robot.')
    self.rtde_c.stopScript()
  
  def run_data_collection(self):
    # Start the data collection process
    self.data_collector.publish_command()
    rclpy.spin(self.data_collector)

def main(args=None):
    rclpy.init(args=args)
    # exp = [-0.575, -0.301, 0.2, -127, 0, 0] # initial position
    exp = [-0.5944156254820065, -0.09571589518438497, -0.08188298887537103, -121.94775197663965, 1, 35]
    task = EEPrimitives(exp)
    pose = task.euler_to_rotvec_pose(exp)
    task.move_l_to_start(pose)
    time.sleep(2)
    task.continuous_move_down(exp)
    rclpy.spin(task)

    task.disconnect()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
