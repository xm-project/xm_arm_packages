#!/usr/bin/env python
#encoding:utf8
import rospy
from smach import State,UserData,Concurrence,StateMachine
from smach_ros import MonitorState
from tf import *
from xm_arm_controller_level import arm_controller_level,lifting_controller_level,gripper_service_level
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from ar_track_alvar_msgs.msg import *
import tf
import math
import subprocess

# 使用该状态机来测试识别并抓取贴有二维码标签的瓶子

class ArmGo(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys=['arm_ps','arm_mode'])
        self.arm_client  = rospy.ServiceProxy('arm_stack',xm_PickOrPlace)
    def execute(self,userdata):
        try:
            getattr(userdata,'arm_ps')
            getattr(userdata, 'arm_mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        req = xm_PickOrPlaceRequest()
        req.action = userdata.arm_mode
        req.goal_position = userdata.arm_ps
        self.arm_client.wait_for_service(timeout=10)#########################################
        res = self.arm_client.call(req)
        rospy.sleep(5.0)#####################################################################
        if res.result == False:
            return 'aborted'
        else:
            return 'succeeded'

class ArGraspTask():
    def __init__(self):
        rospy.init_node('xm_arm_ar')
        rospy.logwarn('Come on! Let us go now!')
        self.arm_stack_service  = rospy.Service('arm_stack',xm_PickOrPlace,self.callback)
        self.sm_ArTags =StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_ArTags:
            self.sm_ArTags.userdata.arm_ps = PointStamped()
            StateMachine.add('GETPOISITION',
                                MonitorState('ar_pose_marker',AlvarMarkers,self.position_cb,max_checks=1,output_keys =['obj_pos']),
                                transitions={'valid':'PICK','invalid':'aborted','preempted':'aborted'},
                                remapping={'obj_pos':'arm_ps'})

            self.sm_ArTags.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmGo(),
                                transitions={'succeeded':'PLACE','aborted':'GETPOISITION','error':'error'},
                                remapping={'arm_ps':'arm_ps','arm_mode':'arm_mode_1'})

            self.sm_ArTags.userdata.arm_mode_2 = 2
            StateMachine.add('PLACE',ArmGo(),
                             transitions={"succeeded":'succeeded','aborted':'aborted','error':'error'},
                             remapping={'arm_ps':'arm_ps','arm_mode':'arm_mode_2'})
        
        outcome = self.sm_ArTags.execute()


    def position_cb(self,userdata,msg):
        if msg.markers is not None:
            obj_pos = PointStamped()
            obj_pos.point = msg.markers[0].pose.pose.position
            obj_pos.header.frame_id ='camera_Link'
            userdata.obj_pos  = obj_pos
            return True
        else:
            return False


    def callback(self,req):
        self.arm_point = req.goal_position
        self.arm_mode  = req.action
        sm_result = self.sm_ArTags.execute(parent_ud ={'arm_point':self.arm_point,'arm_mode':self.arm_mode})
        res = xm_PickOrPlaceResponse()        
        if sm_result == 'succeeded':
            
            rospy.logwarn('arm_stack succeeded')
            res.result =True
        else:
            rospy.logwarn('moveit failed, please justfy the position of robot')
            res.result =False
        return res
                
if __name__ =="__main__":
    try:
        ArGraspTask()
    except KeyboardInterrupt:
        pass
