#!/usr/bin/env python
"""
.. module:: Ontologybuild
:platform: Unix
:synopsis: Python script for building an ontology representation of a map

.. moduleauthor:: Youssef Attia youssef-attia@live.com

This script loads the main ontology file topological_map.owl and adds information about rooms and doors to it. It disjoins the rooms and doors and adds a visitedAt property for each room, indicating the time that the robot visited it. The robot's current location is also updated in the ontology.

The modified ontology is then saved to a separate file and a message is published to the mapsituation topic indicating that the map has been built. This modified ontology can then be used by the finitestates node.
"""

import random
import time
import math
import rospy
import rospkg
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool, String
from surveillance_fsm_robot.srv import RoomInformation
"""
Inherit the package pass and setes the .owl file pass 
"""
r = rospkg.RosPack()
path = r.get_path('surveillance_fsm_robot')
oldontology = path + "/Ontologies/topological_map.owl"
newontology = path + "/Ontologies/my_map.owl"

"""
Global Variables used to set the random sleeping time between each visit, the *maxwait* is set as as the urgent room threshold is 7 and there is 4 wait periods so 7/4 = 1.75 sec
"""
minwait = 0.0
maxwait = 1.75
markers = []


def collectmarkers(string):
   global markers
   for word in string.data.split():
      if word.isdigit():
         num = int(word)
         if 10 < num < 18 and num not in markers:
            markers.append(num)
   

def findtime(list):
   """
   Find the time in Unix format from a list returned by a query for a property from the Armor ontology library.

   Args:
   time (list): A list containing the time in the Armor response format, e.g. ['"1669241751"^^xsd:long']

   Returns:
   time (string): The extracted time as a string, e.g. "1665579740"
   """
   for i in list:
    try:
        start = i.index('"') + len('"')
        end = i.index('"', start)
        return i[start:end]
    except ValueError:
        return ""

def build_Ontology():
   """
   Build an ontology by loading an existing ontology file, adding information about rooms and doors, updating the timestamps for when each room was visited, and saving the modified ontology to a new file.

   Args:
   None

   Returns:
   None
   """

   client = ArmorClient("example", "ontoRef")
   client.call('LOAD','FILE','',[oldontology, 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])

   rospy.wait_for_service('/room_info')
   room_info_service = rospy.ServiceProxy('/room_info', RoomInformation)
   doors_rooms_lsit= []
   for marker in markers:
      response = room_info_service(marker)
      room_name = response.room
      doors_rooms_lsit.append(room_name)
      room_Xcord = response.x
      room_Ycord = response.y
      client.manipulation.add_dataprop_to_ind("Xcoordinates", room_name , "float", str(room_Xcord))
      client.manipulation.add_dataprop_to_ind("Ycoordinates", room_name , "float", str(room_Ycord))
      for i in response.connections:
         i.through_door
         doors_rooms_lsit.append(i.through_door)
         client.call('ADD','OBJECTPROP','IND',['hasDoor', room_name, i.through_door])

   #Disjointing the rooms and doors
   client.call('DISJOINT','IND','',list(set(doors_rooms_lsit)))
   client.call('REASON','','',[''])


   #Adding the visitedAt property for each room
   for i in list(set(doors_rooms_lsit)):
      if i.startswith('R'):
         client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))
         rospy.sleep(random.uniform(minwait, maxwait))

   #Visit E
   #Update robot isin property
   client.call('REASON','','',[''])
   client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
   #Update robot now property
   client.call('REASON','','',[''])
   req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
   oldtimerobot=findtime(req.queried_objects)
   newtime=str(math.floor(time.time()))
   client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])

   client.call('SAVE','','',[newontology])


def main():
   """
   The main function of the script. It initializes a ROS node, creates a subscriber and a publisher, and sets up a service client. It then enters a loop that waits for a message to be received on the subscriber, processes the message, and publishes a response. The loop also sleeps for a random amount of time before processing the next message.

   Args:
   None

   Returns:
   None
   """
   rospy.init_node('mapsituation_node', anonymous=True)
   subscriber=rospy.Subscriber("/marker_publisher/chatter", String, collectmarkers)
   while True:
      print("waiting for markers, markers found:")
      print(markers)
      if len(markers)>=7:
         print("I have all the markers, I will build the map")
         subscriber.unregister()
         pub = rospy.Publisher('mapsituation', Bool, queue_size=10)
         pub.publish(0)
         build_Ontology()
         print("Map built, I will shutdown the node")
         pub.publish(1)
         rospy.sleep(3)
         rospy.signal_shutdown('mapsituation_node')
      rospy.sleep(1)

   
if __name__ == '__main__':
   try:
      main()
   except rospy.ROSInterruptException:
      pass 