#! /usr/bin/env python3

"""
This is a node to test COLIFT state
"""

import rospy
from std_msgs.msg import Int64, String
import time
from Classes.move_thread_class import MoveThread
from Classes.robot_commander_class import RobotCommander

import queue
import threading
import sys

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

# rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
# rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
robot_commander = RobotCommander(start_node=False)

test_count = Int64()
dir_str = String()
prev_dir_str = String()
dir_str.data = "d"
prev_dir_str.data = "s"
dir_change_flag = False


def cb_dir_int(msg):
    global dir_str, prev_dir_str, dir_change_flag
    dir_str = msg
    if prev_dir_str.data == dir_str.data:
        # dir_change_flag = False
        pass
    else:
        dir_change_flag = True
        prev_dir_str = dir_str
        print("dir changed")


def update(pub_test_cmd):
    # force_apply()
    test_count.data = test_count.data + 1
    pub_test_cmd.publish(test_count)
    print(test_count)



def func(q, goal):
    while True:
        task = q.get()
        robot_commander.rtde_c.moveL(goal)
        q.task_done()
        print(f'Thread is doing task #{task} in the queue.')




def main():
    global dir_change_flag
    rospy.init_node('move_test')
    print("MoveL/MoveJ test node started")
    pub_test_cmd = rospy.Publisher('/test_in', Int64, queue_size=1)
    sub_dir_cmd = rospy.Subscriber('/dir_str', String, cb_dir_int)
    rate = rospy.Rate(100)


    release_pose = robot_commander.rtde_c.getForwardKinematics(robot_commander.release_joints, robot_commander.tcp_offset)
    before_release_pose = release_pose.copy()
    before_release_pose[2] += 0.3
    after_release_pose = release_pose.copy()
    after_release_pose[1] += -0.1
    target_poses = [before_release_pose, release_pose, after_release_pose]

    
    # move_thread = MoveThread(rtde_r=robot_commander.rtde_r, rtde_c=robot_commander.rtde_c, goal=release_pose)
    
    
    
    
    
    # release_pose = robot_commander.rtde_c.getForwardKinematics(robot_commander.release_joints, robot_commander.tcp_offset)
    # robot_commander.rtde_c.moveL(release_pose)
    # robot_commander.open_gripper()
    # print("Robot at RELEASE")
    ## check game_end_param set
    
    # robot_commander.rtde_c.moveL(after_release_pose)
    # print("Robot at RELEASE APPROACH")
    # game_over_flag.data = True

    


    # robot_commander.rtde_c.servoStop()
    
    # move_thread.join() # this blocks until the process terminates

    try:
        while not rospy.is_shutdown():
            update(pub_test_cmd)
            if robot_commander.rtde_c.isSteady():
                print("Robot steady")
                if len(target_poses) == 0:
                    sys.exit()
                elif len(target_poses) == 1:
                    print("open grippper")
                    sys.exit()
                else:
                    print(target_poses)
                    robot_commander.rtde_c.moveL(target_poses[0], 0.25, 1.2, True)
                    target_poses.pop(0)
                rospy.sleep(0.5)
            else:
                pass
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        robot_commander.rtde_c.servoStop()
        raise

    robot_commander.rtde_c.servoStop()
    print("Done")


if __name__ == '__main__':
    main()



