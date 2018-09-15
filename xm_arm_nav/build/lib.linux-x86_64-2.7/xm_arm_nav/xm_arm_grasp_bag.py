#!/usr/bin/env python
# encoding:utf8
import rospy
from xm_msgs.srv import *
from xm_msgs.msg import *
from smach import State,UserData,StateMachine
from geometry_msgs.msg import *
import tf
import math
import subprocess
from control_msgs.msg import *
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from xm_arm_controller_level import arm_controller_level, lifting_controller_level, gripper_service_level
from xm_arm_moveit_level import xm_arm_moveit_level, xm_arm_moveit_name
from std_srvs.srv import Empty,EmptyRequest
from copy import deepcopy

object_name = 'bag'

middle_joints = list()
grasp_joints = list()
class FindObject(State):
    def __init__(self):
        State.__init__(self,
                         outcomes=['succeeded','aborted','error'],
                         input_keys=['name'],
                         output_keys =['object_pos'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        goal = Point()
        try:
            name = userdata.name
        except:
            rospy.logerr('No param specified')
            return 'error'
        # subprocess.call("xterm -e rosrun xm_vision image_test &",shell=True)
        self.xm_findobject.wait_for_service(timeout=60.0)
        
        for i in range(3):
            try:

                req = xm_ObjectDetectRequest()
                req.object_name = name
                res = self.xm_findobject.call(req)
                if len(res.object)!=0:
                    break
            except:
                return 'aborted'
        if i==2:
            return 'aborted'
        #   object_pos is PointStamped
        self.tf_listener.waitForTransform('base_link','camera_Link',rospy.Time(),rospy.Duration(60.0))
        rospy.logwarn('tf wait succeeded')

        # pay attention to this, this is caused by the low-accuracy of the arm, 
        # if the arm-control is accurate, the 2 ways following may be all ok
         
        # object_pos = self.tf_listener.transformPoint('base_link',res.object[0].pos)
        # rospy.logwarn('tf transform succeeded')
        # # the pos may be some different...
        # object_pos.point.y = -object_pos.point.y
        # object_pos.point.y -=0.125
        # object_pos.point.x -=0.11
        # object_pos.point.z =0.02
        object_pos = PointStamped()
        (tran,rot) = self.tf_listener.lookupTransform('base_link','camera_Link',rospy.Time(0))
        object_pos.point.x = res.object[0].pos.point.z -0.16
        object_pos.point.y = res.object[0].pos.point.x -0.10
        object_pos.point.z = 0.917-res.object[0].pos.point.y 
        object_pos.header.frame_id = 'base_link'
        
        userdata.object_pos = object_pos
        print object_pos
        
        return 'succeeded'

class ArmBag(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                            input_keys=['arm_ps'])
       
        self.ik_client = rospy.ServiceProxy('xm_ik_solver',xm_SolveIK)
        self.tf_listener =tf.TransformListener()

    def execute(self,userdata):
        global middle_joints, grasp_joints
        try:
            getattr(userdata,'arm_ps')
            userdata.arm_ps.point.z += 0.07
            userdata.arm_ps.point.x += 0.0
            userdata.arm_ps.point.y += 0.0
        except:
            rospy.logerr('No params specified')
            return 'error'
        service_bool =self.ik_client.wait_for_service(timeout=10)
        if service_bool ==False :
            rospy.logerr('time out, ik-service is no avaiable')
            return 'aborted'
        else:
            # this is the first motion
            rospy.logwarn('first step')
            ik_req = xm_SolveIKRequest()
            ik_req.goal = deepcopy(userdata.arm_ps)
            ik_res = self.ik_client.call(ik_req)
            ik_bool =True
            self.ik_client.close()
            if ik_res.result ==True:
                solution_list = list(ik_res.solution)          
                lifting_value = solution_list[0]
                joint_value= solution_list[1:]
                grasp_joints = deepcopy(solution_list)
                arm_controller_level(joint_value)
                # move the lifting 
                lifting_controller_level(lifting_value)
                rospy.sleep(1.0)
            else:
                ik_bool = False
                rospy.logwarn('ik error')

            # this is the second motion, will open or close the gripper
            # true open false close
            rospy.logwarn('second step')
            gripper_mode  = False
            gripper_service_level(gripper_mode)
            rospy.sleep(1.0)
            if ik_bool ==True:

                rospy.logwarn('third step')
                joint_value[2] +=0.3
                joint_value[3] -=0.2
                arm_controller_level(joint_value)
                # move the lifting 
                lifting_controller_level(lifting_value)
                rospy.sleep(1.0)
            else:
                pass

            rospy.logwarn('fourth step')
            xm_arm_moveit_name('after_grasp_bag')
            rospy.sleep(1.0)
            return 'succeeded'
class GraspTask():
    def __init__(self):
        global object_name
        rospy.init_node('grasp_task')
        xm_arm_moveit_name('nav_pose2')
        rospy.logwarn('grasp test is beginning')
        self.sm_Grasp = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.sm_Grasp:
            self.sm_Grasp.userdata.arm_ps = PointStamped()
            
            self.sm_Grasp.userdata.name = object_name
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                remapping={'name':'name','object_pos':'arm_ps'},
                                transitions={'succeeded':'ARMBAG','aborted':'FIND_OBJECT','error':'error'})

            StateMachine.add('ARMBAG',
                                ArmBag(),
                                remapping={'arm_ps':'arm_ps'},
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            # self.sm_Grasp.userdata.move_point = Point(-0.1,0.0,0.0)
            # StateMachine.add('MOVE_BACK',
            #                     SimpleMove(),
            #                     remapping={'point':'move_point'},
            #                     transitions={'succeeded':'PLACE','aborted':'PLACE','error':'error'})
        outcome = self.sm_Grasp.execute()


if __name__ =="__main__":
    try:
        GraspTask()
    except KeyboardInterrupt:
        pass
            
            
