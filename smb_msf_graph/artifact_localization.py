#!/usr/bin/env python

import rospy 
import numpy as np
import math
import csv


from object_detection_msgs.msg import   PointCloudArray,ObjectDetectionInfo, ObjectDetectionInfoArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
import tf

# GLOBALS:
confidence_threshold = 0.4
distance_threshold = 1.0 # Absolute distance between same-label artifacts threshold (meters)

detected_obj = ObjectDetectionInfo()

detected_object_list = [] # Contains Class_id and Global position of detected objects
detected_obs_dict = {}


# Computes the distance between two detected objects
def compute_distance(obj,detected_obj):
    return math.sqrt((obj.position.x-detected_obj.position.x)**2 + (obj.position.y-detected_obj.position.y)**2 + (obj.position.z-detected_obj.position.z)**2)



def object_to_world(relative_pos_obj, translation, rotation):
    # TODO: Convert the object from relative pos wrt robot to global
    # RETURN: detected_object
    res = rotation @ np.array([relative_pos_obj.x, relative_pos_obj.y, relative_pos_obj.z, 1])
    # res = rotation_matrix @ 
    return res


# Stores the new object in the detected_object_list
def store_object(detected_obj):
    added_same_label_object = False

    if len(detected_object_list) == 0:
        detected_object_list.append(detected_obj)
        detected_obs_dict[detected_obj.class_id] = [detected_obj] 
        return
    if detected_obj.class_id not in detected_obs_dict.keys():
        detected_object_list.append(detected_obj)
        detected_obs_dict[detected_obj.class_id] = [detected_obj]
    else:
        distances = [compute_distance(detected_obj, obj) for obj in detected_obs_dict[detected_obj.class_id]]
        print(distances)
        if min(distances) < distance_threshold:
            return
        else:    
            detected_object_list.append(detected_obj)
            detected_obs_dict[detected_obj.class_id].append(detected_obj) 

    return
    #1st check: If there is the same class detected again
    
    # for obj in detected_object_list:
    #     # print(obj.class_id, detected_obj.class_id)
    #     if obj.class_id == detected_obj.class_id:
    #         # 2nd check: If it is not close to the same-label artifact
    #         # print(compute_distance(obj,detected_obj))
    #         if min(compute_distance(obj,detected_obj)]) > distance_threshold:
    #             detected_object_list.append(detected_obj)
    #             print("added at 1")
    #             added_same_label_object = True
            
    # # If there is no same-label object in the list
    # if detected_obj.class_id not in [obj.class_id for obj in detected_object_list]:
    #     detected_object_list.append(detected_obj)
    #     return


def get_matrix_from_pose(point):
    pose = pose_array.poses[0]
    # get translation and rotation
    translation = [pose.position.x, pose.position.y, pose.position.z]
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    transformation_matrix = tf.transformations.compose_matrix(translate=translation, angles=tf.transformations.euler_from_quarternion(rotation))

    return transformation_matrix

if __name__ =="__main__":
    rospy.init_node("localize_art", anonymous=True)
    ros_rate = rospy.Rate(100) # hz

    tf_listener = tf.TransformListener()    
    possible_labels = ["umbrella","bottle","bawl","stop_sign","backpack","suitcase","clock","chair"]

    stored_objects = {}

    while not rospy.is_shutdown():
        try:
            detected_obj_arr     = rospy.wait_for_message("/object_detector/detection_info", ObjectDetectionInfoArray, timeout=None)
            global_pos_robot = rospy.wait_for_message("/tracking_camera/odom/sample", Odometry , timeout=None)
            tf_listener.waitForTransform('/tracking_camera_odom', '/rgb_camera_optical_link', rospy.Time(), rospy.Duration(0.1))
            (translation, rotation) = tf_listener.lookupTransform('/tracking_camera_odom', '/rgb_camera_optical_link', rospy.Time(0)) 
            # print(translation)
            rotation_matrix = tf.transformations.quaternion_matrix(rotation)
            rotation_matrix[:3, 3] = translation
            # print(rotation_matrix)
            # transform 
            if len(detected_obj_arr.info) > 0:
                for i in range(len(detected_obj_arr.info)):

                    detected_obj = detected_obj_arr.info[i]
                    # if (detected_obj.confidence > confidence_threshold) and (detected_obj.class_id in possible_labels):
                    detected_obj_global = object_to_world(detected_obj.position, translation, rotation_matrix)
                    detected_obj.position.x = detected_obj_global[0]
                    detected_obj.position.y = detected_obj_global[1]
                    detected_obj.position.z = detected_obj_global[2]
                    store_object(detected_obj)
                    # print(detected_obj_final)

            print(len(detected_object_list))
    
        except KeyboardInterrupt:
            print("hahahahahahhaahhahhhahahah")
            object_list = [[obj.class_id, obj.position.x, obj.position.y, obj.position.z] for obj in detected_object_list]
            with open('Objects.csv', 'w') as file:
                write = csv.writer(file)

                write.writerows(object_list)

                
        #     for i in detected_object_list:
        #         print("Class = ", i.class_id, "  x = " , i.position.x, "  y = " , i.position.y, "  z = " , i.position.z)

        
        #     print("===========================")