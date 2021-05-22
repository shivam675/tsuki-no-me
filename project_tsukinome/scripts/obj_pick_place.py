#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import UInt8, Float32MultiArray
#from PySide import QtCore, QtGui, QtOpenGL
from tf import TransformListener
from tf.transformations import quaternion_from_euler
import actionlib
import math
from object_msgs.msg import ObjectPose
import geometry_msgs.msg

class Ur5Moveit:

    # Constructor
    def __init__(self):

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m') 
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        self._planning_group = "arm_group"
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    def go_to_defined_pose(self, Plan_group, arg_pose_name):
        '''prefined pose combined with plan_group to minimise error '''
        self._planning_group = Plan_group
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._group.set_named_target(arg_pose_name)
        rospy.sleep(1)
        # plan_success, plan, planning_time, error_code = self._group.plan() 
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        rospy.sleep(1)
        self._exectute_trajectory_client.wait_for_result()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main(translation_list):
    ur5 = Ur5Moveit()
    # best roll pitch yaw values for arm manipulation 
    roll= -3.12
    pitch = 0.5 
    yaw = 1.59
    for j in range(0,len(translation_list),2):

        rospy.sleep(0.5)
        ur5_pose_1 = geometry_msgs.msg.Pose() 
        ur5_pose_1.position.x = translation_list[j+1][0] 
        ur5_pose_1.position.y = translation_list[j+1][1] -0.22 # to get better approach in Y  
        ur5_pose_1.position.z = translation_list[j+1][2] +0.035 # to get better path planning in Z

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        ur5_pose_1.orientation.x = quaternion[0]
        ur5_pose_1.orientation.y = quaternion[1]
        ur5_pose_1.orientation.z = quaternion[2]
        ur5_pose_1.orientation.w = quaternion[3]
        rospy.sleep(0.5)
        
        # rospy.loginfo("Attemp in Y")
        ur5.go_to_pose(ur5_pose_1)    # Attemp to goal in y direction
        ur5_pose_1.position.y += 0.13  #only move in y axis 
        # rospy.loginfo("Attemp in dot")
        ur5.go_to_pose(ur5_pose_1)  # to exact location with small tolerance
        ur5.go_to_defined_pose("end_group","grip_close_3") # Activating gripper pose
        # rospy.sleep(1)
        ur5_pose_1.position.z += 0.19 #going away from object in Z
        # rospy.loginfo("Attemp in Z")
        ur5.go_to_pose(ur5_pose_1)   #going away from object in Z
        
        ur5.go_to_defined_pose("arm_group","drop_box_1") # go to Predefined pose # to dropbox
        ur5.go_to_defined_pose("end_group","open_grip")  # go to Predefined pose # drop object
        ur5.go_to_defined_pose("arm_group","up_view")  #again to initial position

    del ur5
