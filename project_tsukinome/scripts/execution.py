#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
import moveit_msgs.msg
import move_base_msgs.msg
from action2 import *
from obj_pick_place import *
import geometry_msgs.msg
from tf import TransformListener
from image_view import  *

objects_dropped_list = []
object_picked_rooms = []
current_location = [0,0,0,0] # current location and pose of Ebot 

pickups_and_drop_locations = {
    "Coke" : ["Living Room", "Kitchen"],
    "Bowl" : ["Sink", "Drawing Room"],
}

list_of_rooms = ["Kitchen", "Drawing Room"]

obj_id = {
    58 : ["Coke", None],
    # 48 : ["Bowl", None],
    # 48 : ["Glass", None],
} # { "object id" : ["object name linked", "other property(not used)"}

pickup_rooms = {
    "Kitchen" : [[[7.93, -2.70, -1.571]]],
    "Drawing Room" : [[[-8.25, -3.45, 0]]],
}

dropdown_rooms = {
    "Living Room" : [1.21, -0.86, 3.142],
    "Sink" : [7.23, -4.20, 3.142],
} # DropBox Locations

def bounding_box_gui(detection_info):
    """Drawing Bounding Boxes Around objects"""
    object_detection_list = []
    detection_info = detection_info.data

    for i in range(0,len(detection_info),12):   #taking iterations with 13 items at once
        obj_name = obj_id[detection_info[i]][0]   # from ID to Object Name
        x = detection_info[i+9]                   # x-ordinate
        y = detection_info[i+10]                  # y-ordinate
        width = detection_info[i+1]               # width of object
        height = detection_info[i+2]              # height of object  
        object_detection_list.append([x,y,width,height,obj_name])
        rospy.loginfo('\033[94m' + obj_name + " Identified" +'\033[0m')
    
    obj_bound(object_detection_list)

def attemp_pickup(translation_list, pre_location, room_name, detection_info):

    for j in range(0,len(translation_list),2):
        global objects_dropped_list
        object_name = obj_id[translation_list[j]][0]   # from ID to Object Name
        drop_box_room = pickups_and_drop_locations[object_name][1]
        dropbox_location = dropdown_rooms[drop_box_room]

        if object_name in objects_dropped_list:  # if object already picked and placed
            break

        # rospy.loginfo('\033[94m' + object_name + " Identified" +'\033[0m')

        # if j+1 < len(translation_list) and j!= 0:
        #     # nav = MoveBase()
        #     if room_name == "Meeting Room":
        #         nav.movebase_client(pickup_rooms[room_name][2])
        #         nav.movebase_wait()

        #     nav.movebase_client(pre_location)
        #     nav.movebase_wait()
        #     # pass
            
        roll, pitch, yaw = -3.12, 0.5, 1.59

        rospy.sleep(0.5)
        ur5_pose = geometry_msgs.msg.Pose() 
        ur5_pose.position.x = translation_list[j+1][0] 
        ur5_pose.position.y = translation_list[j+1][1] -0.22 # to get better approach in Y  
        ur5_pose.position.z = translation_list[j+1][2] +0.035 # to get better path planning in Z

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        ur5_pose.orientation.x = quaternion[0]
        ur5_pose.orientation.y = quaternion[1]
        ur5_pose.orientation.z = quaternion[2]
        ur5_pose.orientation.w = quaternion[3]
        rospy.sleep(0.5)
        
        # rospy.loginfo("Attemp in Y")
        ur5.go_to_defined_pose("arm_group","up_view")  # go to Predefined pose # to pre-pickup-pose
        ur5.go_to_pose(ur5_pose)    # Attemp to goal in y direction
        ur5_pose.position.y += 0.13  #only move in y axis 
        # rospy.loginfo("Attemp in dot")

        ur5.go_to_pose(ur5_pose)  # to exact location with small tolerance
        if object_name == 'Battery':
            ur5.go_to_defined_pose("end_group","grip_close_2") # Activating gripper pose
        else:
            ur5.go_to_defined_pose("end_group","grip_close_1") # Activating gripper pose
        # rospy.sleep(1)
        ur5_pose.position.z += 0.25 #going away from object in Z
        # rospy.loginfo("Attemp in Z")
        ur5.go_to_pose(ur5_pose)   #going away from object in Z

        ur5_pose.position.y -= 0.25  #only move in y axis 
        ur5.go_to_pose(ur5_pose)  # to exact location with small tolerance
        
        rospy.loginfo('\033[94m' + object_name + " Picked" + '\033[0m')
        ur5.go_to_defined_pose("arm_group","up_view")  # go to Predefined pose # to pre-pickup-pose
        ur5.go_to_defined_pose("arm_group","move_view")  #again to initial position
        
        # if room_name == "Meeting Room":
        #     nav.movebase_client(pickup_rooms[room_name][1])
        #     nav.movebase_wait()
            
        nav.movebase_client(dropbox_location)
        nav.movebase_wait()
        rospy.loginfo('\033[94m' + drop_box_room + " Reached" +'\033[0m')

        # ur5.go_to_defined_pose("arm_group","drop_box_pre")  # go to Predefined pose # to pre-dropbox-pose
        if drop_box_room == "Living Room":
            ur5.go_to_defined_pose("arm_group","drop_coke") # go to Predefined pose # to dropbox
        else:
            ur5.go_to_defined_pose("arm_group","drop_bowl")
        
        ur5.go_to_defined_pose("end_group","open_grip")  # go to Predefined pose # drop object
        rospy.loginfo('\033[94m' + object_name +" Dropped in " + pickups_and_drop_locations[object_name][0] + '\033[0m')

        objects_dropped_list.append(object_name)
        object_picked_rooms.append(room_name)
        # ur5.go_to_defined_pose("arm_group","drop_box_pre")  # go to Predefined pose # to pre-dropbox-pose
        ur5.go_to_defined_pose("arm_group","move_view") # moving pose
        # rospy.sleep(7)

def find_object_msg(msg):
    
    LIST_OF_POSE = []
    objectNumList = [58,59] #only targeted objects  #list of objects defined in find Object 2d pkg 
    data = msg.data
    
    for i in range(0,len(data),12):
        id = int(data[i])

        if id in objectNumList : 
            pass 
        else:
            rospy.loginfo("idle mode") 
            continue

        h = TransformListener()

        rospy.sleep(0.5)
        (trans,rot) = h.lookupTransform('ebot_base', 'object_'+str(id), rospy.Time()) #transformation w.r.t ebot_base

        LIST_OF_POSE.append(id)
        LIST_OF_POSE.append(trans)
    complete_list = LIST_OF_POSE
    return complete_list

if __name__ == '__main__':

    """ Things Start here """

    rospy.init_node('master_node', anonymous=True)
    rospy.loginfo('\033[94m'+"Started Run!" + '\033[0m')

    rospy.sleep(0.3)

    ur5 = Ur5Moveit()
    nav = MoveBase()  

    for i in list_of_rooms :
      for j in pickup_rooms[i][0]:
        
        # if len(objects_dropped_list) == 3 :
        #     break
        
        ur5.go_to_defined_pose("arm_group","move_view") # moving pose


        # if i == "Kitchen":
        #     nav.movebase_client(pickup_rooms[i][2])
        #     nav.movebase_wait()

        nav.movebase_client(j)
        nav.movebase_wait()
        rospy.loginfo('\033[94m' + str(i)+" Reached" + '\033[0m')
        # ur5.go_to_defined_pose("arm_group","front_view")
        ur5.go_to_defined_pose("arm_group","up_view") # Detection view

        detection_info = rospy.wait_for_message('objects', Float32MultiArray, timeout=None) #taking detected objects info from find obj 2d pkg
        list_of_pose = find_object_msg(detection_info)
        
        bounding_box_gui(detection_info)
        attemp_pickup(list_of_pose,j,i,detection_info)

    ur5.go_to_defined_pose("arm_group","move_view")
    nav.movebase_client([0,0,-3.14])  # Returning To Starting Point
    nav.movebase_wait()           # Wait untill Ebot Reach Destination
    rospy.loginfo('\033[94m' + "Mission Accomplished!" + '\033[0m')
