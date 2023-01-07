#!/usr/bin/env python
"""
Publishes the battery state to the topic batterylevel.

If the battery is low, it cancels any existing goals and publishes 0 to the batterylevel topic.
After a random sleep time between charegedtime_min and charegedtime_max, it publishes 1 to the batterylevel topic, indicating that the battery is charged.
"""


import rospy
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import random

charegedtime_min = 100
charegedtime_max = 300
def cancel_move_base_goal():
    """
    Cancels any existing goals for the move_base action server.
    """
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
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
