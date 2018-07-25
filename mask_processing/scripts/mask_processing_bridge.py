#!/usr/bin/env python  

import rospy
import roslib
import sensor_msgs.msg  
import tf
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
# from std_msgs.msg import Bool
from mask_processing.msg import IntMatrix
from mask_processing.msg import RoadCenterCorection
from camera_front_to_orto_shape_analysis import orto_shape_analysis
# import mask_processing.msg

import numpy as np

# from mask_processing.msg import *

# TODO cange to ROS params
tf_topic_name = '/tf'
camera_info_topic_name = '/wide/CameraInfo'
camera_tf_link = 'cam1_link'
mask_topic_name = ''
Y_distance_lim = 5
cam_h_fov_deg = 110 #TODO calculate real FOV 
cam_v_fov_deg = 50 #TODO calculate real FOV

class MaskProcessingNode:

    def __init__(self, mask_topic, cam_imfo_topic, cam_tf_link):
        #TODO check if is is goot to init the node in the constructor
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
        # the full grojection matrix of the camera
        self.p_matrix_ = camera_info_msg.P
        # asumes focal lengths are equal for x and y in pixels
        self.focal_length_ = camera_info_msg.P[0] 
        #[principal points_x, principal_point_y] in pixels
        self.principal_point_ = [camera_info_msg.P[2], camera_info_msg.P[6]] 
 
        
        #TODO check self.base_to_cam_rot_ is in rad and assert pitch is axis down direction
        im__shape = [self.mask_height_, self.mask_width_,1]
        cam__height = self.base_to_cam_trans_[2]
        cam__pitch__deg = np.rad2deg(self.base_to_cam_rot_[1])

        #a functor that analyzez the mask
        self.mask_analyzer = orto_shape_analysis(im__shape, cam_h_fov_deg/2, cam_v_fov_deg/2, \
          axis_down_direction=cam__pitch__deg, \
          H=cam__height, Y_dist_limit=Y_distance_lim)
    

    def get_mask_height(self):
        return self.mask_height_
    
    def get_mask_width(self):
        return self.mask_width_

    def get_focal_length(self):
        return self.focal_length_

    def get_principal_point(self):
        return self.principal_point_

    def get_base_to_cam_translation(self):
        return self.base_to_cam_trans_

    def get_base_to_cam_rpy(self):
        return self.base_to_cam_rot_

    def get_p_matrix(self):
        return self.p_matrix_


    #TODO add Mask proccong method
    def sashas_mask_processing_method(self, mask):
        """
        content
        """
        # [tan_angle, road_center_offset] in of the camera system
        slope_in_cam_sys, offset_in_cam_sys = self.mask_analyzer(mask)
        theta , d_r = self.get_road_angle_and_offset_in_car_system_from_center_line(slope_in_cam_sys, offset_in_cam_sys)
        road_center_publisher = rospy.Publisher('road_center_correction', RoadCenterCorection)
        msg = RoadCenterCorection(theta , d_r)
        road_center_publisher.publish(msg)
    
    """
    This is the math of tranforming the line given from 
    sasha in sasha's cam coordinats, to a non coordinate parameres
        angle from the heading of the vehicle to the road direction in rads
        delta_r distance from the base of the vehicle to the mid road in meters

    delta_r is positive when road center is to the right of car center
    heading_to_road_angle_rad is positive when the road is turning right in vehicle view
    """
    def get_road_angle_and_offset_in_car_system_from_center_line(self, slope, dr):
        new_slope = slope #camera and base axis systems are colinears
        heading_to_road_angle_rad = np.arctan(new_slope)
        dx = self.base_to_cam_trans_[1] # x in sasha's cam coordinate system is our system's y
        dy = self.base_to_cam_trans_[0] # y in sasha's cam coordinate system is our system's x 
        delta_r = (dr - new_slope*dy + dx)*np.cos(np.arctan(new_slope))
        return [heading_to_road_angle_rad, delta_r]


    def run(self):
        mask_sub = rospy.Subscriber(mask_topic_name, IntMatrix, self.sashas_mask_processing_method)
        rospy.spin()
        pass



if __name__ == "__main__":
    mask_processing_node = MaskProcessingNode(mask_topic_name, camera_info_topic_name, camera_tf_link)
    mask_processing_node.run()
    
