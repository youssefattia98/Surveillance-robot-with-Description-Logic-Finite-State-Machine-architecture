#!/usr/bin/env python

"""
   This script is responsible for moving the arm of the robot in order to scan the room for markers.
   It subscribes to the '/m2wr/joint1_position_controller/state' topic to get feedback on the current position of the arm and
   publishes to the '/m2wr/joint1_position_controller/command' topic to control the movement of the arm.
   The arm is moved from its initial position to -3.14 radians and then back to 1.5 radians before returning to its initial position at 0 radians.
"""

import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

"""
   Global variable to store the current position of the arm.
"""

feedback = 0

def callback(msg):
   """
   Global variable to store the current position of the arm.
   """
   global feedback
   feedback = msg.process_value

def arm_move():
   """
   Function for moving the robot arm in a scanning motion. The arm moves from its initial position to -3.14 radians and then back to 1.5 radians.
   The position of the arm is published to the '/m2wr/joint1_position_controller/command' topic and is subscribed to the '/m2wr/joint1_position_controller/state'
   topic to get feedback on the current position of the arm. The function is terminated after the arm has returned to its initial position.
   """
   print("Moving arm, to scan around the room for markers")
   pub = rospy.Publisher('/m2wr/joint1_position_controller/command', Float64, queue_size=10)
   rospy.Subscriber("/m2wr/joint1_position_controller/state", JointControllerState, callback)
   rospy.init_node('arm_controller', anonymous=True)
   pub.publish(-3.14)
   while feedback > -3.14:
      pub.publish(-3.14)
   pub.publish(1.5)
   while feedback < 1.5:
      pub.publish(1.5)
   pub.publish(0.0)
   print("Scanned room, setting arm to initial position and terminating nodes")
   rospy.signal_shutdown('marker_publisher')

   
if __name__ == '__main__':
   try:
      arm_move()
   except rospy.ROSInterruptException:
      pass 