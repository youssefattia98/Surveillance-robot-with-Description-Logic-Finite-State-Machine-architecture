# Surveillance robot with Description Logic & Finite State Machine architecture.  


# [Sphinx](https://youssefattia98.github.io/surveillance_fsm_robot/)

# 1) Introduction
This ROS package is an extension of the [fsm_robot package](https://github.com/youssefattia98/fsm_robot), with added features for autonomous navigation. The robot is spawned in a simulated world and scans for markers using [OpenCV](https://github.com/ros-perception/vision_opencv.git) and [ARUCO ros](https://github.com/CarmineD8/aruco_ros.git). The marker information is then passed to a server called marker_server, which replies with details about the locations on the map. This information is used to build the ontology, which is used by the finite state machine (FSM) to control the robot's movement to desired coordinates. The FSM is implemented using the [Smach package](http://wiki.ros.org/smach).

The robot's movement is controlled using [gmapping](http://wiki.ros.org/gmapping) and [move_base](http://wiki.ros.org/move_base). The package also depends on the Armor package.

The package was developed on a [docker image](https://hub.docker.com/r/carms84/exproblab) with all necessary dependencies pre-installed. Detailed documentation for the scripts can be found [here](https://youssefattia98.github.io/surveillance_fsm_robot/), generated using Sphinx.

# 2) Software Architecture 
## I) Robot Behavior:  
Firstly the robot is spawned in (-6.0,11.0,0.05) it then starts scanning for thr Aruco markers around which are used to build the Ontology.The robot waits for the ontology (map) to be built in Room E. Then starting from the (move_in_corridor) state it checks if the battery is not low or there is no urgent room, it moves randomly in the two corridors and wait for some time. However, if a battery is low it goes to the state (charging), which keeps the robot in room E and stays there until the battery is charged. Also, if there is an urgent room while the battery is charged the robot visits it and stays there for some time (visitroom state). All the robot movements is controlled using move_base which depends on the gmapping node. Furthermore, whenever the robot reaches a room it scans it using the LIDAR with rotating the robot with a 180 degrees.
The following diagram shows the map Ontology that the robot builds:  
![immagine](https://github.com/youssefattia98/surveillance_fsm_robot/blob/main/docs/Digrams%20%26%20videos/MAP.PNG)

## II) Finite State Machine diagram:  
The following finite state machine shows the behavior the robot follows when the architecture was initially designed:  
![immagine](https://github.com/youssefattia98/surveillance_fsm_robot/blob/main/docs/Digrams%20%26%20videos/fsm_digram.PNG)

## III) Nodes diagram:    
The following diagram shows the software architecture of the package. 

![immagine](https://github.com/youssefattia98/surveillance_fsm_robot/blob/main/docs/Digrams%20%26%20videos/block%20digram.jpg)

# 3) Installation
For setting up the environment for this package to run correctly [Armor package](https://github.com/EmaroLab/armor), [Smach package](http://wiki.ros.org/smach), [OpenCV](https://github.com/ros-perception/vision_opencv.git), [ARUCO ros](https://github.com/CarmineD8/aruco_ros.git), [gmapping](http://wiki.ros.org/gmapping) and [move_base](http://wiki.ros.org/move_base) so please check their documentation for the installation.

Please note the ARUCO ros marker_publisher original file does not publish the markers ID, so this node was modified to publish the ID. Therefore it is required after installation of ARUCO ros package to replace the marker_publisher file with the one provided in this package and surely do not forget to build the package with catkin_make.

On the other hand, *xtrem* is needed as the launch file requires, along with *konsole* they can be installed from the following command:  
```bash
$ sudo apt-get -y install xterm
$ sudo apt-get -y install konsole
``` 
After the *xtrem* and konsole are installed, clone this repo in your ROS workspace and build it using catkin_make as following:
```bash
$ cd <your ROS ws/src>
$ git clone "https://github.com/youssefattia98/surveillance_fsm_robot.git"
$ cd ..
$ catkin_make
```
As the package is built successfully, now it is possible to run the launch files using the bash script in the launch folder
```bash
$ roscd surveillance_fsm_robot/launch
$ bash run.sh
```

# 4) Package In Action  
## I) Autonomous simulation video


https://user-images.githubusercontent.com/69837845/211162545-ec0dd02b-9178-411a-a17a-560b46e78ffc.mp4


  


## II) Rqt diagram:  

The following diagram shows the rqt graph of the package running and how the nodes communicate with each other. Moreover, the package behaviors is as same as the designed software architecture design and mentioned in the above point *2)III)*. The following commands can be used to view the nodes rqt diagram:

```bash
$ sudo apt-get install ros-noetic-rqt
$ rosrun rqt_graph rqt_graph
```

![immagine](https://github.com/youssefattia98/surveillance_fsm_robot/blob/main/docs/Digrams%20%26%20videos/rqt_grapgh.jpg)

# 5) Working Hypothesis & Environment
## I) System’s Features
* The package is able to showcase the capabilities of smach package combined with armor package.
* The package is able to detect aruco markers using aruco package and opencv.
* The package is able to implement the concept of surveillance robot using gmapping and move_base.
* The package runs the robot in autonomous mode to visit urgent rooms, charge itself when needed scan the room when reached.
## II) System’s Limitations
* The package is designed to build the map as presented before, therefore the behavior of the robot is according to the specified map. Therefore, if another map ontology is built the package will not work properly. This is due to my limited understanding of the armor package documentation. 

* As mentioned before, the package launch files operates the robot in autonomous mode there for the user has not input for the battery state or map situation state. However, all the states was tested and the video above in point *4)I)* shows so.

## III) Future Improvements
* Creating a manual mode launch file:  
    Applying such solution would give the user the capability to design his/her own map and pass it to the package, also gives the capability of testing different situations to see how the robot behaves.
* creating a separate node for urgency checking:  
    Creating such a node would be a better software architecture as the *finitestate* node will be only responsible for the robot behavior and the urgency will be passed by a message through a topic that the **finitestate* node subscribes to. 
 
* Tune the Move_base parameters to work better.

# 6) Authors and contacts
* Name: Youssef Attia
* Email: youssef-attia@live.com
* LinkedIn: https://www.linkedin.com/in/youssefattia98/
