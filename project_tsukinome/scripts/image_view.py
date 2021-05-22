#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf import TransformListener
import cv2
import numpy as np

bridge = CvBridge()

def callBack(msg,list_of_points):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #success, img = cv_image.read()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    rospy.logwarn_once('Conversion_succesful')
    # blob = cv.dnn.blobFromImage(img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
    # net.setInput(blob)
    # layersNames = net.getLayerNames()
    # outputNames = [(layersNames[i[0] - 1]) for i in net.getUnconnectedOutLayers()]
    # outputs = net.forward(outputNames)
    # findObjects(outputs,img)
    # cv.imshow('Image', img)
    # rospy.loginfo('')
    bounding_box(img,list_of_points)

def bounding_box(img, list_of_points):
    rospy.loginfo('msg _must be displayed rn')

    # print(img)

    # convert to grayscale
    # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # threshold
    # thresh = cv2.threshold(gray,128,255,cv2.THRESH_BINARY)[1]

    # get contours

    result = img.copy()
    ########################################################
    for Object in list_of_points:

        x = int(Object[0])
        y = int(Object[1])
        w = int(Object[2])
        h = int(Object[3])
    
        text        = Object[4]
        font        = cv2.FONT_HERSHEY_SIMPLEX
        location    = (x,y)
        fontScale   = 1
        fontColor   = (0,255,0)
        lineType    = 2

        cv2.rectangle(result, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(result,text, 
                    location, 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)


    ########################################################


    # x,y,20,20 = cv2.boundingRect()
    # print('x,y,w,h:',x,y,w,h)

    # save resulting image
    result     

    # show thresh and result  
    cv2.imshow('Objects Detected', result)
    cv2.waitKey(4000)
    # rospy.sleep(1)
    cv2.destroyAllWindows()


# if __name__=="__main__":

def obj_bound(list_of_points):
    
    rospy.sleep(1)
    sub = rospy.wait_for_message('/camera/color/image_raw2', Image, timeout=2)
    callBack(sub, list_of_points)
    rospy.logwarn('Done !')
