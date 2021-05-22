#!/usr/bin/env python

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBase():

    def __init__(self):

        # rospy.init_node('move_base_sequence')
        self.target = []

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        # rospy.loginfo("Connected to move base server")
        # rospy.loginfo("Starting goals achievements ...")
        # self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.target)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.target)+": "+str(feedback))
        # rospy.loginfo("Feedback for goal pose "+str(feedback)+" received")
        return feedback

    def done_cb(self, status, result):
        # self.target += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.target)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.target)+" reached") 
            # rospy.signal_shutdown("exiting MoveBase")

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.target)+" was aborted by the Action Server")
            # rospy.signal_shutdown("Goal pose "+str(self.target)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.target)+" has been rejected by the Action Server")
            # rospy.signal_shutdown("Goal pose "+str(self.target)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.target)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self, target):

        self.target = target
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose.position.x = self.target[0] #self.pose_seq[self.target][0] 
        goal.target_pose.pose.position.y = self.target[1] #self.pose_seq[self.target][1]

        quaternion = quaternion_from_euler(0, 0, self.target[2])

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("Sending goal pose "+str(self.target)+" to Action Server")
        # rospy.loginfo(str(self.pose_seq[self.target]))
        return self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        # rospy.spin()

    def movebase_response(self):
        return self.client.get_result()
    
    def movebase_wait(self):
        return self.client.wait_for_result()

# if __name__ == '__main__':
#     try:
#         MoveBaseSeq()
#         print("going good..")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation finished.")