#!/usr/bin/env python
"""
.. module:: batterynode
    :platform: Unix
    :synopsis: Python module for publishing the battery state to the topic (batterylevel)

.. moduleauthor:: Youssef Attia youssef-attia@live.com
"""


import rospy
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import random

charegedtime_min = 100
charegedtime_max = 300
def cancel_move_base_goal():
    # Create a simple action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Wait for the action server to become available
    client.wait_for_server()
    # Cancel any existing goals
    client.cancel_all_goals()

def talker():
    """
    Function to initiate a topic *batterylevel* to publish the battery situation.  

    Args:
        void  
        
    Returns:
        void
    """
    pub = rospy.Publisher('batterylevel', Bool, queue_size=10)
    rospy.init_node('batterylevel_node', anonymous=True)
    while not rospy.is_shutdown():
        batt = 0
        print("Battery is low, please charge the robot")
        print("Cancling goals, to charge the robot")
        cancel_move_base_goal()
        pub.publish(batt)
        sleeptime=random.uniform(charegedtime_min, charegedtime_max)
        rospy.sleep(sleeptime)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
