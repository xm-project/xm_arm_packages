#!/usr/bin/env python
#encoding:utf8
'''
/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
'''

# Description: This script can control the arm's position in the rviz.

# Create Date: 2015.11.1

# Authors: myyerrol  


import sys
import tty
import termios
import select
import rospy
import tf
import actionlib
import math
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from std_msgs.msg import String,Int32,Bool
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from math import *
import actionlib
#from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal,FollowJointTrajectoryAction
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

goal_joint_position = [0.0,0.0,0.0,0.0,0.0,0.0]
flag = 2

class KeyboardTeleopRvizArm:
    setting = termios.tcgetattr(sys.stdin)

    def __init__(self):
        rospy.init_node('xm_keyboard_teleop_rviz_arm', anonymous=False)     #节点注册
        client = actionlib.SimpleActionClient('/mobile_base/xm_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client.wait_for_server()
        client_lifting = actionlib.SimpleActionClient('/mobile_base/xm_lifting_controller/lifting_cmd', FollowJointTrajectoryAction)
        client_lifting.wait_for_server()
        joint_pos_step = rospy.get_param('joint_pos_step', 0.01745)
        print "--------- Teleop arm in the rviz ----------"
        print "Q(R)              W(T)                 E(Y)"
        print "A(F)                                   D(H)"
        print "                  S(G)                     "
        print "-------------------------------------------"
        print "W:Lift-Up S:Waist-Left A:big_arm-Up        "
        print "D:Forearm-Up Q:Wrist-Up           "
        print "E:Small_Arm-Clock--                   "
        print "T:Lift-Down G:Waist-Right F:big_arm-Down   "
        print "H:Forearm-Down R:Wrist-Down       "
        print "Y:Small_Arm-Clock++                   "
        print "Z:Exit!!!                                  "
        try:
            global flag
            while True:
                keyboard = self.get_key()#获取键值
                if keyboard == 'w':
                    flag = 1
                    goal_joint_position[0] += 0.03
                    if goal_joint_position[0] >= 0.09:
                        goal_joint_position[0] = 0.09
                elif keyboard == "t":
                    flag = 1
                    goal_joint_position[0] -= 0.03
                    if goal_joint_position[0] <= -0.18:
                        goal_joint_position[0] = -0.18
                elif keyboard == 's':
                    flag = 0
                    goal_joint_position[1] += 0.15
                    if goal_joint_position[1] >= 1.57:
                        goal_joint_position[1] = 1.57
                elif keyboard == 'g':
                    flag = 0
                    goal_joint_position[1] -= 0.15
                    if goal_joint_position[1] <= -0.35:
                        goal_joint_position[1] = -0.35
                elif keyboard == 'a':
                    flag = 0
                    goal_joint_position[2] += 0.3
                    if goal_joint_position[2] >= 1.57:
                        goal_joint_position[2] = 1.57
                elif keyboard == 'f':
                    flag = 0
                    goal_joint_position[2] -= 0.3
                    if goal_joint_position[2] <= -1.57:
                        goal_joint_position[2] = -1.57
                elif keyboard == 'd':
                    flag = 0
                    goal_joint_position[3] += 0.3
                    if goal_joint_position[3] >= 2.0:
                        goal_joint_position[3] = 2.0
                elif keyboard == 'h':
                    flag = 0
                    goal_joint_position[3] -= 0.3
                    if goal_joint_position[3] <= -2.0:
                        goal_joint_position[3] = -2.0
                elif keyboard == 'q':
                    flag = 0
                    goal_joint_position[4] += 0.4
                    if goal_joint_position[4] >= 2.2:
                        goal_joint_position[4] = 2.2
                elif keyboard == 'r':
                    flag = 0
                    goal_joint_position[4] -= 0.4
                    if goal_joint_position[4] <= -2.2:
                        goal_joint_position[4] = -2.2
                elif keyboard == 'e':
                    flag = 0
                    goal_joint_position[5] += 0.3
                    if goal_joint_position[5] >= 1.57:
                        goal_joint_position[5] = 1.57
                elif keyboard == 'y':
                    flag = 0
                    goal_joint_position[5] -= 0.3
                    if goal_joint_position[5] <= -1.57:
                        goal_joint_position[5] = -1.57
                elif keyboard == 'z':
                    exit()
                else:
                    pass
                self.send_joint_state(client, client_lifting)
        except Exception as exce:
            print "Error!", exce
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)

    def send_joint_state(self, client, client_lifting):
        global flag
#        print (flag)
        if flag is 2:
            pass
        elif flag is 0:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names.append('joint_waist')
            goal.trajectory.joint_names.append('joint_big_arm')
            goal.trajectory.joint_names.append('joint_fore_arm')
            goal.trajectory.joint_names.append('joint_wrist')
            goal.trajectory.joint_names.append('joint_small_arm')
            temp_joint=JointTrajectoryPoint()
            temp_joint.positions=[goal_joint_position[1],goal_joint_position[2],goal_joint_position[3],goal_joint_position[4],goal_joint_position[5]]
            temp_joint.accelerations=[0.0,0.0,0.0,0.0,0.0]
            temp_joint.velocities=[0.0,0.0,0.0,0.0,0.0]
            temp_joint.effort=[0.0,0.0,0.0,0.0,0.0]
            temp_joint.time_from_start=rospy.Duration(secs=2.0)
            goal.trajectory.points.append(temp_joint)
            goal.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(secs=1)
            client.send_goal(goal)
            flag = 2
            #result =False
            #while result ==False:
             #   result =  client.wait_for_result(rospy.Duration.from_sec(1.0))

        elif flag is 1:
            goal_lifting = FollowJointTrajectoryGoal()
            goal_lifting.trajectory.joint_names.append('joint_lifting')
            temp_joint_lifting=JointTrajectoryPoint()
            temp_joint_lifting.positions=[goal_joint_position[0]]
            temp_joint_lifting.accelerations=[0.0]
            temp_joint_lifting.velocities=[0.0]
            temp_joint_lifting.effort=[0.0]
            temp_joint_lifting.time_from_start=rospy.Duration(secs=2.0)
            goal_lifting.trajectory.points.append(temp_joint_lifting)
            goal_lifting.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(secs=1)
            client_lifting.send_goal(goal_lifting)
            flag = 2
            #client_lifting.wait_for_result(rospy.Duration.from_sec(2.0))


    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.setting)
        return key


if __name__ == '__main__':
    try:
        KeyboardTeleopRvizArm()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Error!")
