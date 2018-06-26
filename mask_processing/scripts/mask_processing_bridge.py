#!/usr/bin/env python  

import rospy
import roslib
import sensor_msgs.msg  
import tf
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool
from mask_processing.msg import BoolMatrix

# import numpy as np

# from mask_processing.msg import *


# TODO cange to ROS params
tf_topic_name = '/tf'
camera_info_topic_name = '/wide/CameraInfo'
camera_tf_link = 'cam1_link'
mask_topic_name = ''
way_no_way_freq = 10

class MaskProcessingNode:

    def __init__(self, mask_topic, cam_imfo_topic, cam_tf_link):
        rospy.init_node("mask_processing_node")
        rospy.loginfo("Starting MaskProcessingNode.")
        
        transformer = tf.Transformer() #TODO check the arguments
        (trans,rot) = transformer.lookupTransform(cam_tf_link,'base_link',rospy.Time(0))
        # translation vector from base_link to cam1_link
        self.base_to_cam_trans_ = trans
        # rotation vector from base_link to cam1_link in rpy
        self.base_to_cam_rot_ = euler_from_quaternion(rot)

        # sensor_msgs.msg.CameraInfo camera_info_msg
        camera_info_msg = rospy.wait_for_message(cam_imfo_topic, sensor_msgs.msg.CameraInfo)

        self.mask_height_ = camera_info_msg.height
        self.mask_width_ = camera_info_msg.width
        # asumes focal lengths are equal for x and y in pixels
        self.focal_length_ = camera_info_msg.P[0] 
        #[principal points_x, principal_point_y] in pixels
        self.principal_point_ = [camera_info_msg.P[2], camera_info_msg.P[6]] 
    

    def get_height(self):
        return self.mask_height_
    
    def get_width(self):
        return self.mask_width_

    def get_focal_length(self):
        return self.focal_length_

    def get_principal_point(self):
        return self.principal_point_

    def get_base_to_cam_translation(self):
        return self.base_to_cam_trans_

    def get_base_to_cam_rpy(self):
        return self.base_to_cam_rot_


    #TODO add Mask proccong method
    def sashas_mask_processing_method(self, mask):
        """
        content
        """

        pass

    def run(self):
        mask_sub = rospy.Subscriber(mask_topic_name, BoolMatrix, self.sashas_mask_processing_method)
        rospy.spin()



if __name__ == "__main__":
    mask_processing_node = MaskProcessingNode(mask_topic_name, camera_info_topic_name, camera_tf_link)
    mask_processing_node.run()
    
