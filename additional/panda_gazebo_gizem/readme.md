# PANDA with GAZEBO and Moveit

This guy is great: https://erdalpekel.de/?p=55#comment-4 even new fork for panda is using his solution.

This repo https://github.com/justagist/panda_simulator has gazebo features but they use their own HW transmission, which is not a position control, which is not set as action server (as far as I know).

There was also a repo claims that gazebo works with moveit for Panda. It didn't work for me. (It can be a branch in the default repo but I closed the tab and I cant find it right now).



**Problem** is that MoveIT expect a follow joint trajectory from the robot which is not published by any official repositories of Panda. Here I changed the transmissions and controllers and mine works like a gem <3



**NOTE!** Name the package properly without "Gizem" and publish it. For now, located under `/home/gizem/catkin_ws/src/imu_human_pkg/additional/panda_gazebo_gizem` 



## STEPS:

Put the necessary files as shown here:

```
├── CMakeLists.txt
├── config
│   └── arm_controller_panda.yaml
├── description
│   ├── hand.urdf.xacro
│   ├── hand.xacro
│   ├── panda_arm.urdf.xacro
│   ├── panda_arm.xacro
│   ├── panda_gazebo.xacro
│   └── utils.xacro
├── launch
│   └── panda_gizem.launch  # This one is created based on panda.launch in franka_gazebo package
├── package.xml
├── readme.md
└── src

4 directories, 11 files
```



1) Remove franka transmissions in `panda_arm.urdf.xacro`

   ```
       <!-- <xacro:transmission-franka-state arm_id="${arm_id}" />
       <xacro:transmission-franka-model arm_id="${arm_id}"
          root="${arm_id}_joint1"
          tip="${arm_id}_joint8"
        /> -->
   ```

2) Change gazebo plugin type

```
    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>${arm_id}</robotNamespace>
        <controlPeriod>0.001</controlPeriod>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      </plugin>
      <self_collide>true</self_collide>
    </gazebo>
```

3. Change transmission types in `utils.xacro`

   ```
     <xacro:macro name="gazebo-joint" params="joint transmission:=hardware_interface/PositionJointInterface">
       <gazebo reference="${joint}">
         <!-- Needed for ODE to output external wrenches on joints -->
         <provideFeedback>true</provideFeedback>
       </gazebo>
   
       <transmission name="${joint}_transmission">
         <type>transmission_interface/SimpleTransmission</type>
         <joint name="${joint}">
           <hardwareInterface>${transmission}</hardwareInterface>
         </joint>
         <actuator name="${joint}_motor">
           <!-- <hardwareInterface>${transmission}</hardwareInterface> -->
           <mechanicalReduction>1</mechanicalReduction>
         </actuator>
       </transmission>
     </xacro:macro>
   ```

   The **transmission-franka-state**, **transmission-gizem-state** and **transmission-franka-model** can stay but I am not using them. I can clean them later.

4. Load controllers:

   a. Create yaml in /config

   ```
   panda:                    #useful if you use a namespace for the robot
       # Publish joint states
       joint_state_controller:
           type: joint_state_controller/JointStateController
           publish_rate: 50
   
       panda_arm_controller:
           type: effort_controllers/JointTrajectoryController
           joints:
               - panda_joint1
               - panda_joint2
               - panda_joint3
               - panda_joint4
               - panda_joint5
               - panda_joint6
               - panda_joint7
   
           gains:
               panda_joint1: { p: 12000, d: 50, i: 0.0, i_clamp: 10000 }
               panda_joint2: { p: 30000, d: 100, i: 0.02, i_clamp: 10000 }
               panda_joint3: { p: 18000, d: 50, i: 0.01, i_clamp: 1 }
               panda_joint4: { p: 18000, d: 70, i: 0.01, i_clamp: 10000 }
               panda_joint5: { p: 12000, d: 70, i: 0.01, i_clamp: 1 }
               panda_joint6: { p: 7000, d: 50, i: 0.01, i_clamp: 1 }
               panda_joint7: { p: 2000, d: 20, i: 0.0, i_clamp: 1 }
   
           constraints:
               goal_time: 2.0
   
           state_publish_rate: 25
   
       panda_hand_controller:
           type: effort_controllers/JointTrajectoryController
           joints:
               - panda_finger_joint1
               - panda_finger_joint2
   
           gains:
               panda_finger_joint1: { p: 5, d: 3.0, i: 0, i_clamp: 1 }
               panda_finger_joint2: { p: 5, d: 1.0, i: 0, i_clamp: 1 }
   
           state_publish_rate: 25
   ```

   b. Load in the launch as rosparam

   ```
   <rosparam file="$(find panda_gazebo_gizem)/config/arm_controller_panda.yaml" command="load"/>
   ```

   **NOTE:** Try to load it as one of the first thing in the launch because sometimes controller starts before rosparam is ready. DON'T LOAD JUST BEFORE YOU START CONTROLLERS!

5. Remove other controller spawners and just put this:

6. ```
   <!-- Spawn required ROS controllers -->
   <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn joint_state_controller panda_arm_controller panda_hand_controller" respawn="false" output="screen"/>
   ```

   

## How to run

```
roslaunch panda_gazebo_gizem panda_gizem.launch
```

These are then the available topics, arm controller can be used with moveit now!

![](/home/gizem/Insync/giat@hvl.no/Onedrive/Notes/fig/panda_controller_topics.png)

Running controllers:

![](/home/gizem/Insync/giat@hvl.no/Onedrive/Notes/fig/panda_gazebo_running_controllers.png)
