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
from ar_track_alvar_msgs.msg import *
# this state is used for arm-catch task
class ArmGo(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','preempted'],
                        input_keys=['obj_pos'])
        self.ik_client  = rospy.ServiceProxy('xm_ik_solver',xm_SolveIK)
    def execute(self,userdata):
        # make the preempt_requested
        if self.preempt_requested():
            return 'preempted'
        else:
            pass
        self.obj_pos = userdata.obj_pos
        if self.obj_pos.header.frame_id is None:
            rospy.logerr('lack the frame_id of the object_position')
            return 'aborted'
        self.ik_client.wait_for_service(timeout=10.0)
        req  = xm_SolveIKRequest()
        req.goal.header.frame_id = self.obj_pos.header.frame_id
        req.goal.point = self.obj_pos.point
        res = self.ik_client.call(req)
        if res.result == True:
            solution_list = list(res.solution)
            # make the gripper foward to the bottle
            solution_list[2] +=0.3
            solution_list[3] -=0.3
            lifting_value = solution_list[0]
            joint_value= solution_list[1:]
            arm_controller_level(joint_value)
            # move the lifting 
            lifting_controller_level(lifting_value)
            rospy.logdebug('I will go to catch the bottle')

            solution_list = list(res.solution)
            joint_value= solution_list[1:]
            arm_controller_level(joint_value)
            return 'succeeded'
        else:
            return 'aborted'


# simple state use for delay some time
class Wait(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=["succeeded"],
                        input_keys=['rec'])

    def execute(self, userdata):
        rospy.sleep(userdata.rec)
        return "succeeded"


class XmArmArTags():
    def __init__(self):
        rospy.init_node('xm_arm_ar_tags')
        rospy.on_shutdown(self.shutdown)
        # design the main part of the test snippet
        # 
        # self.arm_concurrence = StateMachine(outcomes =['succeeded','aborted','preempted'])
        # 
        # TODO:add the speech-recognize function with the pocketsphinx package
        # this will download from the source in the kinetic
        # http://blog.csdn.net/x_r_su/article/details/53022746?locationNum=1&fps=1
        # with self.arm_concurrence:
        self.sm_ArTags =StateMachine(outcomes =['succeeded','aborted','preempted'])
        with self.sm_ArTags:
            #TODO:here we need to make the topic only return when the ar_tags is detected 
            # and we should do that:the libfreenectgrabber to publish the PointClouds data real-time
            # but make the Octomap only generate for a certain time
            self.sm_ArTags.userdata.object_position = PointStamped()
            StateMachine.add('GETPOISITION',
                                MonitorState('ar_pose_marker',AlvarMarkers,self.position_cb,max_checks=1,output_keys =['obj_pos']),
                                transitions={'valid':'PICK','invalid':'aborted','preempted':'aborted'},
                                remapping={'obj_pos':'object_position'})
            StateMachine.add('PICK',
                                ArmGo(),
                                transitions={'succeeded':'WAIT','aborted':'GETPOISITION','preempted':'aborted'},
                                remapping={'obj_pos':'object_position'})
            self.sm_ArTags.userdata.rec = 2.0
            StateMachine.add('WAIT',
                                Wait(),
                                remapping={'rec':'rec'},
                                transitions={'succeeded':'GETPOISITION'})
        
        outcome = self.sm_ArTags.execute()

    def shutdown(self):
        rospy.logerr('byebye')

    def position_cb(self,userdata,msg):
        if msg.markers is not None:
            obj_pos = PointStamped()
            obj_pos.point = msg.markers[0].pose.pose.position
            obj_pos.header.frame_id ='camera_Link'
            userdata.obj_pos  = obj_pos
            return True
        else:
            return False
                
if __name__ =="__main__":
    try:
        XmArmArTags()
    except KeyboardInterrupt:
        pass
