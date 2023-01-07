#! /usr/bin/env python
"""
.. module:: coordinate_controller
:platform: Unix
:synopsis: Python module for providing a ROS service that moves the robot to the specified coordinates.

.. moduleauthor:: Youssef Attia youssef-attia@live.com
"""

import rospy
from surveillance_fsm_robot.srv import Cordinates_srv
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
 


def SSR(req):
    """
    Function to move the robot to the specified coordinates.

    Copy code
    Args:
        req (Cordinates_srv): A service request containing the x and y coordinates of the target location. 
        
    Returns:
        int: A status code indicating whether the robot reached the target location (1) or not (-1).
    """
    x = req.x
    y = req.y
    print("moving to X: ",x," Y: ",y)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #start movebaseaction
    client.wait_for_server() #waiting for response
    
    #create the goal
    goal = MoveBaseGoal()
    #set the goal parameter
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    #send the goal
    client.send_goal(goal)
    #wait for result
    wait = client.wait_for_result(timeout=rospy.Duration(100.0))
    if not wait:
        #target not reached, calling cancle goal and return
        print("Mission failed, robot didn't reach goal")
        client.cancel_goal()
        return -1
    return 1 #if reahced the target

def coordinates_srv():
    """
    Function to initialize the ROS node and create the Cordinates_srv service.
    """
    print("Autonomous node.")
    rospy.init_node('coordinate_controller') #seting up the node
    s = rospy.Service('cordinates_srv' ,Cordinates_srv ,SSR) #calling server service routine
    print("service ready")
    rospy.spin()

if __name__=="__main__":
    coordinates_srv()