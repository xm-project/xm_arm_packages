#!/usr/bin/env python
#encoding:utf8
import rospy
from smach import State, StateMachine,UserData
from xm_msgs.srv import *
from xm_msgs.msg import *
import math
import tf
from geometry_msgs.msg import *
from control_msgs.msg import *
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name
from std_srvs.srv import Empty,EmptyRequest
import subprocess
from copy import deepcopy

def xm_arm_moveit_wave(pose_name):
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        xm_arm_ = moveit_commander.MoveGroupCommander('xm_arm')
        xm_arm_.set_goal_joint_tolerance(0.01)
        xm_arm_.allow_replanning(True)
        xm_arm_.set_planner_id('SBLkConfigDefault')
        xm_arm_.set_start_state_to_current_state()
        xm_arm_.set_named_target(pose_name)
        xm_arm_.go()
    except:
        return False
        rospy.sleep(1.0)
    else:
        return True

if __name__ == "__main__":
    try:
        while True:
           # xm_arm_moveit_name('wave_pose1')
            gripper_service_level(True)
           # xm_arm_moveit_name('wave_pose2')
            gripper_service_level(False)
    except KeyboardInterrupt:
        pass
