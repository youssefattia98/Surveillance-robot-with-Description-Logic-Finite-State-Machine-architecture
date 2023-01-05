#!/usr/bin/env python

import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64


feedback = 0

def callback(msg):
   global feedback
   feedback = msg.process_value

def arm_move():
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
   print("Scanned room, setting arm to initial position and terminating node")

   
if __name__ == '__main__':
   try:
      arm_move()
   except rospy.ROSInterruptException:
      pass 