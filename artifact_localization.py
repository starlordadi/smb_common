#!/usr/bin/env python

import rospy 
import numpy as np
import math


from object_detection_msgs.msg import   PointCloudArray,ObjectDetectionInfo, ObjectDetectionInfoArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

# GLOBALS:
confidence_threshold = 0.4
distance_threshold = 1 # Absolute distance between same-label artifacts threshold (meters)

detected_obj = ObjectDetectionInfo()

detected_object_list = [] # Contains Class_id and Global position of detected objects


# Computes the distance between two detected objects
def compute_distance(obj,detected_obj):
    return math.sqrt((obj.position.x-detected_obj.position.x)**2 + (obj.position.y-detected_obj.position.y)**2 + (obj.position.z-detected_obj.position.z)**2)



def object_to_world(relative_pos_obj,global_pos_robot):
    # TODO: Convert the object from relative pos wrt robot to global
    # RETURN: detected_object
    return detected_obj


# Stores the new object in the detected_object_list
def store_object(detected_obj):
    added_same_label_object = False

    #1st check: If there is the same class detected again
    for obj in detected_object_list:
        if obj.class_id == detected_obj.class_id:
            # 2nd check: If it is not close to the same-label artifact
            if compute_distance(obj,detected_obj) < distance_threshold:
                detected_object_list.append(detected_obj)
                added_same_label_object = True
    # If there is no same-label object in the list
    if not added_same_label_object:
        detected_object_list.append(detected_obj)


if __name__ =="__main__":
    rospy.init_node("localize_art", anonymous=True)
    ros_rate = rospy.Rate(100) # hz
    
    possible_labels = ["umbrella","bottle","bawl","stop_sign","backpack","suitcase","clock","chair"]

    while not rospy.is_shutdown():
        detected_obj     = rospy.wait_for_message("/detection_info", ObjectDetectionInfo , timeout=None)
        global_pos_robot = rospy.wait_for_message("/graph_msf/est_odometry_odom_imu", Odometry , timeout=None)

        if (detected_obj.confidence > confidence_threshold) and (detected_obj.class_id in possible_labels):
            detected_obj = object_to_world(detected_obj.position,global_pos_robot)
            store_object(detected_obj)

        for i in detected_object_list:
            print("Class = ", i.class_id, "  x = " , i.position.x, "  y = " , i.position.y, "  z = " , i.position.z)

        
        print("===========================")