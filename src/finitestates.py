#!/usr/bin/env python
"""
.. module:: finitestates
:platform: Unix
:synopsis: Python script for implementing a finite state machine for a robot

.. moduleauthor:: Youssef Attia youssef-attia@live.com

This script handles the main behavior of a robot by using a finite state machine. It waits for the ontology (map) to be built, and then enters a loop that transitions between three states: move_in_corridor, visitroom, and charging.

In the move_in_corridor state, the robot moves randomly in the corridors and waits for a certain amount of time if the battery is not low and there are no urgent rooms to visit. If the battery is low, the robot transitions to the charging state, in which it stays in room E until the battery is charged. If there is an urgent room to visit while the battery is charged, the robot transitions to the visitroom state and stays there for a certain amount of time.
"""


import roslib
import random
import math
import time
import rospy
import rospkg
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from armor_api.armor_client import ArmorClient
from surveillance_fsm_robot.srv import Cordinates_srv
from geometry_msgs.msg import Twist


"""
Inherit the package pass and setes the .owl file pass 
"""
r = rospkg.RosPack()
path = r.get_path('surveillance_fsm_robot')
newontology = path + "/Ontologies/my_map.owl"  


"""
Global Variables used to understand the map, battery and urgent room situation. Also, set the sleeping time in each room.  

"""
mapflag = 0
batflag = 1
urgentflag = 1
chargingtime = 2
coordinates = {}

def callbackbattery(data):
    """
    This function is a callback for the batterylevel topic. It updates the batflag global variable based on the state of the battery.

    Args:
        data (Bool): The data received in the message, indicating the state of the battery.

    Returns:
        None
    """
    global batflag
    if data.data == 0:
        batflag = 0

def callbackmap(data):
    """
    This function is a callback for the mapsituation topic. It updates the mapflag global variable based on the state of the map.

    Args:
        data (Bool): The data received in the message, indicating the state of the map.

    Returns:
        None
    """
    global mapflag
    if data.data == 1:
        mapflag = 1
    elif data.data ==0:
        mapflag = 0

def move_base(desired):
    """
    This function moves the robot to the specified location.

    Args:
        desired (str): The name of the location to move the robot to.

    Returns:
        rt: The result of the cordinates_srv service call, indicating whether the target was reached successfully.
    """
    cordinates_srv = rospy.ServiceProxy('cordinates_srv', Cordinates_srv)
    rt=cordinates_srv(coordinates[desired]['X'] , coordinates[desired]['Y'])
    if rt.return_ == 1:
            print("Target reached successfully!")
    else:
            print("Target not reached try again!")
            move_base(desired)
    return rt

def urgentupdate():
    """
    Check if there is an urgent room and set the global variable `urgentflag`, also returns the nearby urgent room.
    
    Returns:
        str: The nearby urgent room according to the robot position in the corridors.
    """
    global urgentflag
    tobetrturned = '0'
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','IND','CLASS',['URGENT'])
    req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req2.queried_objects)
    if oldlocation=='E':
        if random.randint(1, 2)==1:
            moveto('C1')
        else:
            moveto('C2')
        client.call('REASON','','',[''])
        req2=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        oldlocation=findindividual(req2.queried_objects)
    for i in req.queried_objects:
        if oldlocation == 'C1':
            r1time=client.call('QUERY','DATAPROP','IND',['visitedAt', 'R1'])
            r2time=client.call('QUERY','DATAPROP','IND',['visitedAt', 'R2'])
            more_urgent = float(findbt(r1time.queried_objects)) - float(findbt(r2time.queried_objects))
            if "R1" in i and more_urgent <= 0:
                urgentflag = 0
                tobetrturned = 'R1'
                break
            elif "R2" in i and more_urgent > 0:
                urgentflag = 0
                tobetrturned = 'R2'
                break
        elif oldlocation == 'C2':
            r3time=client.call('QUERY','DATAPROP','IND',['visitedAt', 'R3'])
            r4time=client.call('QUERY','DATAPROP','IND',['visitedAt', 'R4'])
            more_urgent = float(findbt(r3time.queried_objects))-float(findbt(r4time.queried_objects))
            if "R3" in i and more_urgent <= 0:
                urgentflag = 0
                tobetrturned = 'R3'
                break
            elif "R4" in i and more_urgent > 0:
                urgentflag = 0
                tobetrturned = 'R4'
                break
    if  tobetrturned == '0':
        urgentflag = 1
    else:
        return tobetrturned

def moveto(newloction):
    """
    This function moves the robot to a new location specified in the argument. It determines the current location of the robot and updates the
    isIn property accordingly. The function also updates the now property and the visitedAt property of the new location.

    NOTE: This function is not generic and is specific to the current ontology and its map architecture. To make it more general, the connectedTo
    property can be utilized.

    Args:
        new_location (str): The location that the robot should be moved to.

    Returns:
        None
    """
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    oldlocation=findindividual(req.queried_objects)
    print('From', oldlocation, 'to: ', newloction)
    canReach = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
    canReach = list_Locations(canReach.queried_objects)


    if newloction in canReach:
        #Update isIn property
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',newloction, oldlocation])
        move_base(newloction)
        client.call('REASON','','',[''])
        
        #Update now data property
        req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(math.floor(time.time())), findbt(req.queried_objects)])
        client.call('REASON','','',[''])
        
        #Update visitedAt data property
        isRoom = client.call('QUERY','CLASS','IND',[newloction, 'true'])
        if isRoom.queried_objects == ['URGENT'] or isRoom.queried_objects == ['ROOM']:
            req=client.call('QUERY','DATAPROP','IND',['visitedAt', newloction])
            client.call('REPLACE','DATAPROP','IND',['visitedAt', newloction, 'Long', str(math.floor(time.time())), findbt(req.queried_objects)])
            client.call('REASON','','',[''])



    else:
        location_connectedTo = client.call('QUERY','OBJECTPROP','IND',['connectedTo', newloction])
        location_connectedTo = list_Locations(location_connectedTo.queried_objects)
        common=find_common_connection(location_connectedTo,canReach)
        if common == None:
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',canReach[0],oldlocation])
            move_base(canReach[0])
            client.call('REASON','','',[''])
            req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            oldlocation=findindividual(req.queried_objects)
            canReach = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
            common=find_common_connection(location_connectedTo,list_Locations(canReach.queried_objects))

        
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',common,oldlocation])
        move_base(common)
        client.call('REASON','','',[''])
        req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        oldlocation=findindividual(req.queried_objects)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',newloction,oldlocation])
        move_base(newloction)
        client.call('REASON','','',[''])
        #Update now data property
        req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(math.floor(time.time())), findbt(req.queried_objects)])
        client.call('REASON','','',[''])
        #Update visitedAt data property
        isRoom = client.call('QUERY','CLASS','IND',[newloction, 'true'])
        if isRoom.queried_objects == ['URGENT'] or isRoom.queried_objects == ['ROOM']:
            req=client.call('QUERY','DATAPROP','IND',['visitedAt', newloction])
            client.call('REPLACE','DATAPROP','IND',['visitedAt', newloction, 'Long', str(math.floor(time.time())), findbt(req.queried_objects)])
            client.call('REASON','','',[''])

    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    print('Robot isIn',findindividual(req.queried_objects))

def scan():
    """
    Function for scanning the environment by rotating the robot in place.

    This function rotates the robot in place by publishing a Twist message with a positive angular.z value to the 'cmd_vel' topic. It does this for 10 iterations at a rate of 10Hz, and then sleeps for 5 seconds.

    Args:
        None

    Returns:
        None
    """
    print("Scanning..")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 1.0
    rate = rospy.Rate(10)
    for i in range(10):
        pub.publish(twist_msg)
        rate.sleep()
    rospy.sleep(5)

def set_coordinates():
    """
    Sets the global variable 'coordinates' to a dictionary of the locations and their corresponding X and Y coordinates.

    First, a list of room names is created. Then, for each room in the list, the Xcoordinates and Ycoordinates of the room are queried using the ArmorClient and stored as variables 'X' and 'Y', respectively. These values are then added to the 'coordinates' dictionary as a key-value pair with the room name as the key and a dictionary of the X and Y coordinates as the value.

    Args:
        void

    Returns:
        void
    """
    global coordinates
    client = ArmorClient("example", "ontoRef")
    list_of_rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    for i in list_of_rooms:
        req=client.call('QUERY','DATAPROP','IND',['Xcoordinates', i])
        X=float(findbt(req.queried_objects))
        req=client.call('QUERY','DATAPROP','IND',['Ycoordinates', i])
        Y=float(findbt(req.queried_objects))
        coordinates[i] = {'X': X, 'Y': Y}

def findindividual(list):
    """
    Function for finding the individual in a list from the return of a qureied proprity from armor.

    This function is used to extract the individual from the format returned by the ArmorClient query, which is a list containing the individual in a URI format.

    Args:
        Individual(list): The list containing the individual in the armor response format, ex. ['http://bnc/exp-rob-lab/2022-23#R1']

    Returns:
        Individual(string): The individual extracted and changed to a string, ex. "R1"
    """
    for i in list:
        if "R1" in i:
            return 'R1'
        elif "R2" in i:
            return 'R2'
        elif "R3" in i:
            return 'R3'
        elif "R4" in i:
            return 'R4'
        elif "C1" in i:
            return 'C1'
        elif "C2" in i:
            return 'C2'
        elif "E" in i:
            return 'E'
def findbt(list):
    """
    Function for extracting data between quotation marks from a list. 

    Args:
        lst (list): A list containing strings with data enclosed in quotation marks. 

    Returns:
        str: The extracted data. 
    """
    for i in list:
        try:
            start = i.index('"') + len('"')
            end = i.index('"', start)
            return i[start:end]
        except ValueError:
            return ""
def list_Locations(list):
    position_list = []
    for i in list:
        if "R1" in i:
            position_list.append('R1')
        elif "R2" in i:
            position_list.append('R2')
        elif "R3" in i:
            position_list.append('R3')
        elif "R4" in i:
            position_list.append('R4')
        elif "C1" in i:
            position_list.append('C1')
        elif "C2" in i:
            position_list.append('C2')
        elif "E" in i:
            position_list.append('E')
    return position_list
def find_common_connection(l1, l2):
  for common in l1:
    if common in l2:
        return common


class waiting_for_map(smach.State):
    """
    Class for state waiting_for_map in which the robot waits for the mapflag to be True and then loads the ontology.

    This state has two possible outcomes:

    keepwaiting if the map is not yet loaded
    maploaded if the map has been successfully loaded
    Once the map has been loaded, the set_coordinates function is called to initialize the coordinates dictionary with the coordinates of each room in the map. The robot is then moved to room E.
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepwaiting','maploaded'])

    def execute(self, userdata):
        global mapflag
        client = ArmorClient("example", "ontoRef")
        if mapflag == 0:
            rospy.sleep(1)
            return 'keepwaiting'
        else:
            client.call('LOAD','FILE','',[newontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            print("MAP IS LOADED...")
            set_coordinates()
            move_base('E')
            return 'maploaded'

class move_in_corridor(smach.State):
    """
    Class representing the state in which the robot checks its battery level and the presence of an urgent room. Depending on the results of these checks, the robot will either continue to move randomly in the corridors, enter the charging state, or visit an urgent room.

    Attributes:
    outcomes (list): possible outcomes of this state, including 'keepmoving' if the battery is not low and there is no urgent room, 'battlow' if the battery is low, and 'urgentvisit' if there is an urgent room.

    Methods:
    execute(self, userdata): carries out the actions associated with this state, including checking the battery level and presence of an urgent room, and returning the appropriate outcome.
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepmoving','battlow','urgentvisit'])

    def execute(self, userdata):
        global batflag
        global urgentflag
        client = ArmorClient("example", "ontoRef")
        urgentupdate()
        if batflag == 0:
            print("BATTERY IS LOW...")
            return 'battlow'
        if urgentflag == 0 and batflag ==1:
            print("THERE IS AN URGENT ROOM...")
            return 'urgentvisit'
        else:
            if random.randint(1, 2)==1:
                moveto('C1')
            else:
                moveto('C2')
            return 'keepmoving'

class charging(smach.State):
    """
    Class for state charging in which the robot moves to room E and stays there until the battery is charged.

    Returns:
        *keepcharging* if the battery is still low, *battfull* if the battery is fully charged, indicated by the global variable *batflag* being set to True.
    """

    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepcharging','battfull'])

    def execute(self, userdata):
        global batflag
        client = ArmorClient("example", "ontoRef")
        if batflag == 1:
            print("BATTERY IS CHARGED...")
            return 'battfull'
        else:
            moveto('E')
            rospy.sleep(chargingtime)
            batflag = 1
            return 'keepcharging'

class visitroom(smach.State):
    """
    Class for state visitroom in which the robot checks the nearby urgent rooms and visit them for a certain amount of time. This state is only reached if the battery is charged and there is an urgent room.

    Returns:
        keepvisiting if the battery is still charged and there are nearby urgent rooms, noturgentvisit if all the nearby urgent rooms have been visited,
        battlow if the battery is low.
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['keepvisiting','noturgentvisit', 'battlow'])

    def execute(self, userdata):
        global batflag
        global urgentflag
        client = ArmorClient("example", "ontoRef")
        urgentupdate()
        if urgentflag == 1:
            return 'noturgentvisit'
        elif batflag ==0:
            return 'battlow'
        else:
            The_urgnet_room=urgentupdate()
            moveto(The_urgnet_room)
            scan()
            return 'keepvisiting'

def main():
    """
    This function initializes the ROS node and creates a finite state machine using the `SMACH package: <http://wiki.ros.org/smach>`_.
    It also subscribes to the 'batterylevel' and 'mapsituation' topics."
    """
    rospy.init_node('Robot_FSM')
    # Create a SMACH state machine
    robot = smach.StateMachine(outcomes=['Interface'])
    # Open the container
    with robot:
        # Add states to the container
        smach.StateMachine.add('waiting_for_map', waiting_for_map(), 
                               transitions={'keepwaiting':'waiting_for_map','maploaded':'move_in_corridor'})
        smach.StateMachine.add('move_in_corridor', move_in_corridor(), 
                               transitions={'keepmoving':'move_in_corridor','battlow':'charging','urgentvisit':'visitroom'})
        smach.StateMachine.add('charging', charging(), 
                               transitions={'keepcharging':'charging','battfull':'move_in_corridor'})
        smach.StateMachine.add('visitroom', visitroom(), 
                               transitions={'keepvisiting':'visitroom','noturgentvisit':'move_in_corridor','battlow':'charging'})
    
    rospy.Subscriber("batterylevel", Bool, callbackbattery)
    rospy.Subscriber("mapsituation", Bool, callbackmap)
    rospy.wait_for_service('cordinates_srv')
    # Execute SMACH plan
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', robot, '/SM_ROOT')
    sis.start()
    outcome = robot.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()  