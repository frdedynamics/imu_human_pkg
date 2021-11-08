# IMU_HUMAN_PKG

This package aims to exploit human movements reliably to be used in HRC (Human-Robot Collaboration) applications. Tracking and estimating human motions, human modelling, human-robot simulations and real-time real world human-robot applications are main subjects.

NOTE: The package is still under development. For any issues please contact the author (gizem.ates@hvl.no) or open an issue. Please welcome to contribute to fix any bugs.


## Motivation

Human body has a lot of information hidden in movements. Just imagine how much you can achive with your mouse, 3 buttons and 2 directional information. Now think about how many gestures, postures and a sequence of these may broaden the possibility of using the human body as a commander (even better, as a collaborative friend to a robot in a harmony of the robot movements as well). The main challenge is to capture and analyse human body motion reliably.

Imagine that you are carrying a table with a friend. For some odd reason neither you or your friend can talk to each other. You don't know which table to carry, or you may even don't know where to place it either. The only thing then to use is the *body language*. You can show which table to be carried, you can push and pull during carrying to direct your friend, you can lift or drop the table from one side and the person on the other side would understand when to pick and when to place.  In this example, we talked as if only one person is *leading* the other one is *following* but let me give an even more edgy example. One of you might know which table to carry (since s/he knows the environment) and the other one might know where to place (since s/he knows the task). In such a scenario, you need to change *role*s smoothly. Remember! You are still not allowed to talk.

Now, replace the scenarios above with a robot and human couple instead of a pair of human couple. Would it change? What to do nothing to be change from the human side? If the robot would *understand* the human friend's movements *as if it was a human* then nothing would have changed on the human side.

Exactly in this point this research comes to the table. Using human body measurements during motion, process them properly and send them to the robot in its language and on top, to achieve it safely/accurately/precisely to be used in real industrial HRC applications, then mission is well accomplished!


### Why not cameras?
In this package only IMU (Inertial Measurement Systems) are used. One may wonder "as a human, I am using my eyes at most. Wouldn't be more natural to use cameras instead? After all, vision systems rocks these days". Well, yes and no.

In HMA (human motion analysis), there are two main categories using visual-based and non-visual-based motion tracking systems(*). Visual-based systems are those which takes RGB and/or depth information of the human body by directly seeing it. Non-visual-based tracking systems do not need to see the human but by simply wearing or attaching the sensory system allow the motion data to be captured and sent to a station or a PC with a wire or wirelessly. In an ideal environment visual based systems perform outstanding. The human motion capture capability can reach up to 60 fps with a regular camera, over 1000 fps with a slow-mo camera, over 100 Hz with a laser scanner. With a Vicon system using 6 cameras and several markers attached on the body, you can get even sub-millimeter accuracy from your motion capture system. However, how much you need to invest on it? How effectively you can use your system in varous conditions. In a perfectly stable lighting conditions, you may get a perfect tracking system but what if some shadows occurs every now and then? Or worse, what happens there is an occlusion on the camera or an obstacle in between your camera and yourself? All these issues can be overcome by using IMUs from non-visual-based motion tracking systems family. Nonetheless, they have some issues still especially when it comes to process acceleration/velocity level data to get position level.



(*)P.S. There are different categorization ways such as wearable/non-wearable, online/offline, with marker/markerless etc.

### Isn't it already there?
This is also yes and no.

We encounter some impressive public videos of robots are moving with humans, human motions are used in animation movies over a decade, robots are dancing etc. There are some systems which solves some problems but not all. If you dig enough t, you may find that the fancy videos we see are mostly the best snap of the system over several trials. It can even seem fancier than actually what is underlying. Therefore in 2021 we haven't surrounded robotic friends around us yet. It requires lots of research still. If you search in the academic literature, you can find many publications addressing these issues.

I tried to make a collection of open-source codes and academic publications (some papers require subscription for full access) related to this study. The lists are constantly being updated.


## Navigating in the Package
The package contains several folders and the descriptions are given there:


-- TODO:

```bash
.
├── action
│   └── handCalibration.action
├── CMakeLists.txt
├── example
├── launch
├── LICENSE.md
├── log
├── package.xml
├── README.md
├── src
├── ui
└── urdf

```

7 directories, 5 files

- /action: custom ROS action/service.
- /src: ROS nodes
- /launch: ROS launch
- /ui: PyQt5 based graphical user interface scripts
- /log: some recorded data
- /config: configuration files such as sensor names, 


## Setup
- `git clone URL` in the src folder in your catkin workspace.
- change directory to the catkin workspace again `cd ~/catkin_ws`
- install dependencies: `rosdep install imu_human_pkg`
- compile workspace: `catkin_make`
- source compiled outputs: `source devel/setup.bash`
- update ROS packages list: `rospack profile`


Then start ROS Master with `roscore` and start the GUI with `rosrun imu_human_pkg main_ui.py`.

## Run
- Change directory to where the main scrip is: `roscd imu_human_pkg/src`
- Start the script: `./qnode.py` or `python3 qnode.py`

### Dependencies
Some necessary dependencies are already installed with `rosdep install imu_human_pkg` command. In addition to those, you need to have 
- [**awindamonitor** for sensors](git clone https://github.com/Raffa87/xsense-awinda.git)
- [If you want to use real UR](https://sdurobotics.gitlab.io/ur_rtde/index.html)
- [If you want to control ROS-i supported simulated robot models in Gazebo](https://moveit.ros.org/robots/)
  - For example (in the paper referred): [Franka Emika Panda in Gazebo controlled by Moveit](https://github.com/frankaemika/franka_ros)


## Useful open-source links

## Useful publications

### HME with IMU
- [Survey of Motion Tracking Methods Based on Inertial Sensors: A Focus on Upper Limb Human Motion](https://www.mdpi.com/1424-8220/17/6/1257)
- [Using Inertial Sensors for Position and Orientation Estimation](https://arxiv.org/pdf/1704.06053.pdf)
- [Xsens MVN : Full 6DOF Human Motion Tracking Using Miniature Inertial Sensors](http://human.kyst.com.tw/upload/pdfs120702543998066.pdf)


### IMU and HRC

- [Human-Robot Cooperative Lifting Using IMUs and Human Gestures](https://link.springer.com/chapter/10.1007%2F978-3-030-89177-0_9)
- [Inertial Human Motion Estimation for Physical Human-Robot Interaction Using an Interaction Velocity Update to Reduce Drift](https://hvlopen.brage.unit.no/hvlopen-xmlui/handle/11250/2583544)

### Miscellaneous
- [Deciding the default human body measurements](http://www.oandplibrary.org/al/pdf/1964_01_044.pdf)
- [Offline human model in Gazebo](https://iopscience.iop.org/article/10.1088/1757-899X/825/1/012006/pdf)
- [How we decide to move - Internal models in the cerebellum](https://www.sciencedirect.com/science/article/pii/S1364661398012212)
- [Are cameras better than IMUs? - Comparison of RGB-D and IMU-based gesture recognition for human-robot interaction in remanufacturing](https://link.springer.com/article/10.1007/s00170-021-08125-9)

