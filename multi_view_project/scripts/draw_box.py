#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 15:09:51 2022

@author: 12106
"""
from __future__ import print_function
from doctest import FAIL_FAST

import cv2
import message_filters
import numpy as np
import resource_retriever
import rospy
import tf.transformations
from PIL import Image
from PIL import ImageDraw
from cv_bridge import CvBridge
# from dope.inference.cuboid import Cuboid3d
# from dope.inference.cuboid_pnp_solver import CuboidPNPSolver
# from dope.inference.detector import ModelData, ObjectDetector
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg
# from std_msgs.msg import String
# from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
# from visualization_msgs.msg import Marker, MarkerArray
import tf
import tf.transformations as transformations
import time
import os
import yaml
import copy

# with open(os.path.expanduser("~/catkin_ws/src/PBPF/config/parameter_info.yaml"), 'r') as file:
#     parameter_info = yaml.safe_load(file)

# object_name_list = parameter_info['object_name_list']
object_num = 2
object_name_list = ['cracker', 'soup']
# the flag is used to determine whether the robot touches the particle in the simulation
simRobot_touch_par_flag = 0
pose_PBPF_list = []
pose_DOPE_list = []
robot_num = 1
class Ros_listener_alg():
    def __init__(self):
        # pose_PBPF = rospy.Subscriber('/esti_obj_list', estimated_obj_pose, self.pose_PBPF_callback, queue_size=1)
        
        name_objs = locals()
        
        # for obj_index in range(object_num):
        #     pose_PBPF = rospy.Subscriber('PBPF_pose_'+object_name_list[obj_index], PoseStamped, self.pose_PBPF_callback, callback_args=obj_index, queue_size=1)
        #     pose_PBPF_list.append(pose_PBPF)
        #     pose_DOPE = rospy.Subscriber('DOPE_pose_'+object_name_list[obj_index], PoseStamped, self.pose_DOPE_callback, callback_args=obj_index, queue_size=1)
            
        pose_PBPF = rospy.Subscriber('PBPF_pose_cracker', PoseStamped, self.pose_PBPF_callback_cracker, callback_args=0, queue_size=1)
        pose_PBPF = rospy.Subscriber('PBPF_pose_soup', PoseStamped, self.pose_PBPF_callback_soup, callback_args=1, queue_size=1)
        
        pose_DOPE = rospy.Subscriber('DOPE_pose_cracker', PoseStamped, self.pose_DOPE_callback_cracker, callback_args=0, queue_size=10)
        pose_DOPE = rospy.Subscriber('DOPE_pose_soup', PoseStamped, self.pose_DOPE_callback_soup, callback_args=1, queue_size=10)
        
            
            
        # pose_Opti = rospy.Subscriber('Opti_pose', PoseStamped, self.pose_Opti_callback, queue_size=1)
        self.current_joint_values = [-1.57,0.0,0.0,-2.8,1.7,1.57,1.1]
        self.PBPF_pos = [ 0.139080286026,
                         -0.581342339516,
                         0.0238141193986]
        #x,y,z,w
        self.PBPF_ori = [ 0.707254290581,
                          0.0115503482521,
                         -0.0140119809657,
                         -0.706726074219]
        
        self.PBPF_pose = [] # [[pos], [ori]]
        self.PBPF_pose.append(self.PBPF_pos)
        self.PBPF_pose.append(self.PBPF_ori)
        self.PBPF_pose_list_temp = [] # [[[pos1], [ori1]],
                                      #  [[pos2], [ori2]] ]
                                      # [obj] [pos/ori] [value]
        
        self.DOPE_pos = [ 0.139080286026,
                         -0.581342339516,
                         0.0238141193986]
        #x,y,z,w
        self.DOPE_ori = [ 0.707254290581,
                          0.0115503482521,
                         -0.0140119809657,
                         -0.706726074219]
        
        self.DOPE_pose = [] # [[pos], [ori]]
        self.DOPE_pose.append(self.DOPE_pos)
        self.DOPE_pose.append(self.DOPE_ori)
        self.DOPE_pose_list_temp = [] # [[[pos1], [ori1]],
                                      #  [[pos2], [ori2]] ]
                                      # [obj] [pos/ori] [value]
        
        for obj_index in range(object_num):
            self.PBPF_pose_list_temp.append(self.PBPF_pose)
            self.DOPE_pose_list_temp.append(self.DOPE_pose)
        
        self.PBPF_pose_list = copy.deepcopy(self.PBPF_pose_list_temp)
        self.DOPE_pose_list = copy.deepcopy(self.DOPE_pose_list_temp)
        
        self.PBPF_pose_cracker = [[],[]]
        self.PBPF_pose_soup = [[],[]]
        self.PBPF_pose_list_test = [[[1,1,1],[1,1,1,1]],[[1,1,1],[1,1,1,1]]]
        
        self.DOPE_pose_cracker = [[],[]]
        self.DOPE_pose_soup = [[],[]]
        self.DOPE_pose_list_test = [[[1,1,1],[1,1,1,1]],[[1,1,1],[1,1,1,1]]]
        
        self.Opti_pos = [ 0.139080286026,
                         -0.581342339516,
                         0.0238141193986]
        #x,y,z,w
        self.Opti_ori = [ 0.707254290581,
                          0.0115503482521,
                         -0.0140119809657,
                         -0.706726074219]

        rospy.spin

    def pose_PBPF_callback_cracker(self, data, obj_index):
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.PBPF_pose_cracker[0] = pos
        self.PBPF_pose_cracker[1] = ori
        self.PBPF_pose_list_test[obj_index] = self.PBPF_pose_cracker
        
    def pose_PBPF_callback_soup(self, data, obj_index):
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.PBPF_pose_soup[0] = pos
        self.PBPF_pose_soup[1] = ori
        self.PBPF_pose_list_test[obj_index] = self.PBPF_pose_soup

    
    def pose_PBPF_callback(self, data, obj_index):
        index = copy.deepcopy(obj_index)
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.PBPF_pose_cracker[0] = pos
        self.PBPF_pose_cracker[1] = ori
        self.PBPF_pose_list[index] = self.PBPF_pose_cracker
        self.PBPF_pose_list_test[index] = self.PBPF_pose_cracker

    def pose_DOPE_callback(self, data, obj_index):
        index = copy.deepcopy(obj_index)
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.DOPE_pose[0] = pos
        self.DOPE_pose[1] = ori
        self.DOPE_pose_list[index] = copy.deepcopy(self.DOPE_pose)
        # self.DOPE_pose_list_test[obj_index] = self.DOPE_pose_soup
    
    
    def pose_DOPE_callback_cracker(self, data, obj_index):
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.DOPE_pose_cracker[0] = pos
        self.DOPE_pose_cracker[1] = ori
        self.DOPE_pose_list_test[obj_index] = self.DOPE_pose_cracker
        
    def pose_DOPE_callback_soup(self, data, obj_index):
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        pos = [x_pos, y_pos, z_pos]
        ori = [x_ori, y_ori, z_ori, w_ori]
        self.DOPE_pose_soup[0] = pos
        self.DOPE_pose_soup[1] = ori
        self.DOPE_pose_list_test[obj_index] = self.DOPE_pose_soup
        
    def pose_Opti_callback(self, data):
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        self.Opti_pos = [x_pos,y_pos,z_pos]
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        self.Opti_ori = [x_ori,y_ori,z_ori,w_ori]
        # print(self.Opti_pos)
        # print(self.Opti_ori)
        
class Ros_listener_OPTI():
    def __init__(self):
        pose_Opti = rospy.Subscriber('Opti_pose', PoseStamped, self.pose_Opti_callback, queue_size=1)
        # print("I am heree")
        self.Opti_pos = [ 0.139080286026,
                         -0.581342339516,
                         0.0238141193986]
        #x,y,z,w
        self.Opti_ori = [ 0.707254290581,
                          0.0115503482521,
                         -0.0140119809657,
                         -0.706726074219]
        rospy.spin

    def pose_Opti_callback(self, data):
        # print("I am hereeeeeeee")
        #pos
        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        z_pos = data.pose.position.z
        self.Opti_pos = [x_pos,y_pos,z_pos]
        #ori
        x_ori = data.pose.orientation.x
        y_ori = data.pose.orientation.y
        z_ori = data.pose.orientation.z
        w_ori = data.pose.orientation.w
        self.Opti_ori = [x_ori,y_ori,z_ori,w_ori]
        # print(self.Opti_pos)
        # print(self.Opti_ori)
        
        
class Ros_listener():
    def __init__(self):
        # Start ROS publishers
        self.pub_rgb_dope_points = \
            rospy.Publisher(
                rospy.get_param('~topic_publishing') + "/rgb_points_my_wireframe",
                ImageSensor_msg,
                queue_size=10
            )
        print(rospy.get_param('~topic_publishing'))
        self.cv_bridge = CvBridge()
        image_sub = message_filters.Subscriber('/camera/color/image_raw', ImageSensor_msg)
        
        ts = message_filters.TimeSynchronizer([image_sub], 1)
        # for obj_index in range(object_num): 
        ts.registerCallback(self.image_callback)
        rospy.spin()
    
    def image_callback(self, image_msg):
        img = self.cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        height, width, _ = img.shape
        img_copy = img.copy()
        im = Image.fromarray(img_copy)
        im = im.convert('RGB')
        draw = Draw(im)
        size_list = []
        cracker_size = [0.065, 0.1067185, 0.0305]
        soup_size = [0.023829689025878906, 0.05, 0.023829689025878906]
        names_obj = locals()
        for obj_index in range(object_num):
            size_list.append(names_obj[object_name_list[obj_index]+"_size"]) # cracker_size/soup_size
        # print("size_list:", size_list)
        
        points_DOPE_list = []
        
        for obj_index in range(object_num):
        
            height = size_list[obj_index][0]
            length = size_list[obj_index][1]
            width = size_list[obj_index][2]
            
            point_0 = [-1 * height, 1 * length, 1 * width]
            point_1 = [ 1 * height, 1 * length, 1 * width]
            point_2 = [ 1 * height,-1 * length, 1 * width]
            point_3 = [-1 * height,-1 * length, 1 * width]
            point_4 = [-1 * height, 1 * length,-1 * width]
            point_5 = [ 1 * height, 1 * length,-1 * width]
            point_6 = [ 1 * height,-1 * length,-1 * width]
            point_7 = [-1 * height,-1 * length,-1 * width]
            rvecs = np.array([[0.0], 
                            [0.0], 
                            [0.0]])
            tvecs = np.array([[0.0], 
                            [0.0], 
                            [0.0]])
            if DOPE_flag == True:
                x_par = 0
                y_par = 0
                # if task_flag == "1" and object_cheezit_flag == True:
                #     x_par = 0
                #     y_par = 7
                # elif task_flag == "1" and object_soup_flag == True:
                #     x_par = 0
                #     y_par = 15
                # elif task_flag == "2":
                #     x_par = 0
                #     y_par = 15
                # elif task_flag == "3":
                #     x_par = 0
                #     y_par = 15
                color = (100, 149, 237) # dark blue
                color = (255, 255, 0) # yellow
                # color = (138, 43, 226) # purple
                # color = (0, 255, 0) # green
                # color = (0, 255, 255) # light blue
                # color = (255, 0, 0) # red
                # while True:
                #     try:
                #         (trans,rot) = listener.lookupTransform('/RealSense', '/cracker', rospy.Time(0))
                #         break
                #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #         continue
                # camera_T_obj_dope_pos = list(trans)
                # camera_T_obj_dope_ori = list(rot)
                # camera_T_obj_dope_3_3 = transformations.quaternion_matrix(camera_T_obj_dope_ori)
                # camera_T_obj_dope_4_4 = rotation_4_4_to_transformation_4_4(camera_T_obj_dope_3_3,camera_T_obj_dope_pos)
                # print(obj_index)
                # print(alg_listener.DOPE_pose_list_test[obj_index])
                
                pw_T_obj_DOPE_pos = copy.deepcopy(alg_listener.DOPE_pose_list_test[obj_index][0])
                pw_T_obj_DOPE_ori = copy.deepcopy(alg_listener.DOPE_pose_list_test[obj_index][1])
                
                # pw_T_obj_DOPE_pos = copy.deepcopy(alg_listener.DOPE_pose_list_test[obj_index][0])
                # pw_T_obj_DOPE_ori = copy.deepcopy(alg_listener.DOPE_pose_list_test[obj_index][1])
                
                # print(obj_index)
                # print(pw_T_obj_DOPE_pos)
                
                if object_name_list[obj_index] == 'cracker':
                    color = (255, 255, 0) # yellow
                elif object_name_list[obj_index] == 'soup':
                    color = (255, 215, 0) # golden
                # pybullet_robot_pos
                # # pybullet_robot_ori
                # print(pw_T_obj_DOPE_pos)
                # print(pw_T_obj_DOPE_ori)
                # print("before")
                while True:
                    try:
                        if optitrack_working_flag == True:
                            (trans,rot) = listener.lookupTransform('/RealSense', '/pandaRobot', rospy.Time(0))
                        else:
                            (trans,rot) = listener.lookupTransform('/ar_tracking_camera_frame', '/panda_link0', rospy.Time(0))
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                # print("after")
                camera_T_rob_dope_pos = list(trans)
                camera_T_rob_dope_ori = list(rot)
                camera_T_rob_dope_3_3 = transformations.quaternion_matrix(camera_T_rob_dope_ori)
                camera_T_rob_dope_4_4 = rotation_4_4_to_transformation_4_4(camera_T_rob_dope_3_3,camera_T_rob_dope_pos)
                pw_T_obj_DOPE_3_3 = transformations.quaternion_matrix(pw_T_obj_DOPE_ori)
                pw_T_obj_DOPE_4_4 = rotation_4_4_to_transformation_4_4(pw_T_obj_DOPE_3_3,pw_T_obj_DOPE_pos)
                pw_T_rob_DOPE_3_3 = transformations.quaternion_matrix(pybullet_robot_ori)
                pw_T_rob_DOPE_4_4 = rotation_4_4_to_transformation_4_4(pw_T_rob_DOPE_3_3,pybullet_robot_ori)
                rob_T_pw_DOPE_4_4 = np.linalg.inv(pw_T_rob_DOPE_4_4)
                camera_T_pw_4_4 = np.dot(camera_T_rob_dope_4_4, rob_T_pw_DOPE_4_4)
                camera_T_obj_dope_4_4 = np.dot(camera_T_pw_4_4, pw_T_obj_DOPE_4_4)
                
                point_ori = [0,0,0,1]
                point_3_3 = transformations.quaternion_matrix(point_ori)
                camera_T_point_0_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_0)
                camera_T_point_1_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_1)
                camera_T_point_2_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_2)
                camera_T_point_3_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_3)
                camera_T_point_4_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_4)
                camera_T_point_5_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_5)
                camera_T_point_6_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_6)
                camera_T_point_7_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_7)
                camera_T_points = []
                camera_T_points.append(camera_T_point_0_pos)
                camera_T_points.append(camera_T_point_1_pos)
                camera_T_points.append(camera_T_point_2_pos)
                camera_T_points.append(camera_T_point_3_pos)
                camera_T_points.append(camera_T_point_4_pos)
                camera_T_points.append(camera_T_point_5_pos)
                camera_T_points.append(camera_T_point_6_pos)
                camera_T_points.append(camera_T_point_7_pos)
                
                results_cv2_points = []
                for i in range(len(camera_T_points)):
                    results_cv2_point = cv2.projectPoints(np.array(camera_T_points[i]),
                                                        rvecs,
                                                        tvecs,
                                                        np.array(_camera_intrinsic_matrix),
                                                        np.array(_dist_coeffs))
                    results_cv2_points.append(tuple([results_cv2_point[0][0][0][0]+x_par,results_cv2_point[0][0][0][1]+y_par]))
                points_DOPE = [results_cv2_points[0], 
                        results_cv2_points[1], 
                        results_cv2_points[2], 
                        results_cv2_points[3], 
                        results_cv2_points[4], 
                        results_cv2_points[5], 
                        results_cv2_points[6], 
                        results_cv2_points[7]]
                t_middle = time.time()
                # print(t_middle - t_begin)
                
                points_DOPE_list.append(points_DOPE)
                
                draw.draw_cube(points_DOPE, color)
                
            if PBPF_flag == True:
                x_par = 10
                y_par = -15
                # if task_flag == "1":
                #     x_par = 0
                #     y_par = -15
                # elif task_flag == "2":
                #     x_par = 10
                #     y_par = -20
                # elif task_flag == "3" and object_soup_flag == True:
                #     x_par = 10
                #     y_par = 0
                # elif task_flag == "3" and object_cheezit_flag == True:
                #     x_par = 10
                #     y_par = -20
                # elif task_flag == "4":
                #     x_par = 0
                #     y_par = 0
                color = (0, 255, 0) # green
                
                pw_T_obj_PBPF_pos = copy.deepcopy(alg_listener.PBPF_pose_list_test[obj_index][0])
                pw_T_obj_PBPF_ori = copy.deepcopy(alg_listener.PBPF_pose_list_test[obj_index][1])
                
                # pw_T_obj_PBPF_pos = alg_listener.PBPF_pose_list[obj_index][0]
                # pw_T_obj_PBPF_ori = alg_listener.PBPF_pose_list[obj_index][1]
                
                if object_name_list[obj_index] == 'cracker':
                    color = (0, 255, 0) # green
                elif object_name_list[obj_index] == 'soup':
                    color = (0, 128, 0) # dark green
                # pybullet_robot_pos
                # pybullet_robot_ori
                # print(pw_T_obj_PBPF_pos)
                # print(pw_T_obj_PBPF_ori)
                while True:
                    try:
                        if optitrack_working_flag == True:
                            (trans,rot) = listener.lookupTransform('/RealSense', '/pandaRobot', rospy.Time(0))
                        else:
                            (trans,rot) = listener.lookupTransform('/ar_tracking_camera_frame', '/panda_link0', rospy.Time(0))
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                camera_T_rob_dope_pos = list(trans)
                camera_T_rob_dope_ori = list(rot)
                camera_T_rob_dope_3_3 = transformations.quaternion_matrix(camera_T_rob_dope_ori)
                camera_T_rob_dope_4_4 = rotation_4_4_to_transformation_4_4(camera_T_rob_dope_3_3,camera_T_rob_dope_pos)
                pw_T_obj_PBPF_3_3 = transformations.quaternion_matrix(pw_T_obj_PBPF_ori)
                pw_T_obj_PBPF_4_4 = rotation_4_4_to_transformation_4_4(pw_T_obj_PBPF_3_3,pw_T_obj_PBPF_pos)
                pw_T_rob_PBPF_3_3 = transformations.quaternion_matrix(pybullet_robot_ori)
                pw_T_rob_PBPF_4_4 = rotation_4_4_to_transformation_4_4(pw_T_rob_PBPF_3_3,pybullet_robot_ori)
                rob_T_pw_PBPF_4_4 = np.linalg.inv(pw_T_rob_PBPF_4_4)
                camera_T_pw_4_4 = np.dot(camera_T_rob_dope_4_4, rob_T_pw_PBPF_4_4)
                camera_T_obj_dope_4_4 = np.dot(camera_T_pw_4_4, pw_T_obj_PBPF_4_4)
                
                point_ori = [0,0,0,1]
                point_3_3 = transformations.quaternion_matrix(point_ori)
                camera_T_point_0_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_0)
                camera_T_point_1_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_1)
                camera_T_point_2_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_2)
                camera_T_point_3_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_3)
                camera_T_point_4_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_4)
                camera_T_point_5_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_5)
                camera_T_point_6_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_6)
                camera_T_point_7_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_7)
                camera_T_points = []
                camera_T_points.append(camera_T_point_0_pos)
                camera_T_points.append(camera_T_point_1_pos)
                camera_T_points.append(camera_T_point_2_pos)
                camera_T_points.append(camera_T_point_3_pos)
                camera_T_points.append(camera_T_point_4_pos)
                camera_T_points.append(camera_T_point_5_pos)
                camera_T_points.append(camera_T_point_6_pos)
                camera_T_points.append(camera_T_point_7_pos)
                
                results_cv2_points = []
                for i in range(len(camera_T_points)):
                    results_cv2_point = cv2.projectPoints(np.array(camera_T_points[i]),
                                                        rvecs,
                                                        tvecs,
                                                        np.array(_camera_intrinsic_matrix),
                                                        np.array(_dist_coeffs))
                    results_cv2_points.append(tuple([results_cv2_point[0][0][0][0]+x_par,results_cv2_point[0][0][0][1]+y_par]))
                points_PBPF = [results_cv2_points[0], 
                        results_cv2_points[1], 
                        results_cv2_points[2], 
                        results_cv2_points[3], 
                        results_cv2_points[4], 
                        results_cv2_points[5], 
                        results_cv2_points[6], 
                        results_cv2_points[7]]
                draw.draw_cube(points_PBPF, color)
            if Opti_flag == True:
                x_par = 0
                y_par = 0
                # if task_flag == "1b":
                #     x_par = 60
                #     y_par = 50
                # elif task_flag == "1a":
                #     x_par = 55
                #     y_par = 75
                # elif task_flag == "2":
                #     x_par = 55
                #     y_par = 75
                color = (0, 255, 255) # light blue
                pw_T_obj_Opti_pos = OPTI_listener.Opti_pos
                pw_T_obj_Opti_ori = OPTI_listener.Opti_ori
                # pybullet_robot_pos
                # pybullet_robot_ori
                # print("pw_T_obj_PBPF_pos:", pw_T_obj_Opti_pos)
                # print("pw_T_obj_PBPF_ori:", pw_T_obj_Opti_ori)
                while True:
                    try:
                        if optitrack_working_flag == True:
                            (trans,rot) = listener.lookupTransform('/RealSense', '/pandaRobot', rospy.Time(0))
                        else:
                            (trans,rot) = listener.lookupTransform('/ar_tracking_camera_frame', '/panda_link0', rospy.Time(0))
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                camera_T_rob_dope_pos = list(trans)
                camera_T_rob_dope_ori = list(rot)
                camera_T_rob_dope_3_3 = transformations.quaternion_matrix(camera_T_rob_dope_ori)
                camera_T_rob_dope_4_4 = rotation_4_4_to_transformation_4_4(camera_T_rob_dope_3_3,camera_T_rob_dope_pos)
                pw_T_obj_Opti_3_3 = transformations.quaternion_matrix(pw_T_obj_Opti_ori)
                pw_T_obj_Opti_4_4 = rotation_4_4_to_transformation_4_4(pw_T_obj_Opti_3_3,pw_T_obj_Opti_pos)
                pw_T_rob_Opti_3_3 = transformations.quaternion_matrix(pybullet_robot_ori)
                pw_T_rob_Opti_4_4 = rotation_4_4_to_transformation_4_4(pw_T_rob_Opti_3_3,pybullet_robot_ori)
                rob_T_pw_Opti_4_4 = np.linalg.inv(pw_T_rob_Opti_4_4)
                camera_T_pw_4_4 = np.dot(camera_T_rob_dope_4_4, rob_T_pw_Opti_4_4)
                camera_T_obj_dope_4_4 = np.dot(camera_T_pw_4_4, pw_T_obj_Opti_4_4)
                
                point_ori = [0,0,0,1]
                point_3_3 = transformations.quaternion_matrix(point_ori)
                camera_T_point_0_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_0)
                camera_T_point_1_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_1)
                camera_T_point_2_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_2)
                camera_T_point_3_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_3)
                camera_T_point_4_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_4)
                camera_T_point_5_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_5)
                camera_T_point_6_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_6)
                camera_T_point_7_pos = point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point_7)
                camera_T_points = []
                camera_T_points.append(camera_T_point_0_pos)
                camera_T_points.append(camera_T_point_1_pos)
                camera_T_points.append(camera_T_point_2_pos)
                camera_T_points.append(camera_T_point_3_pos)
                camera_T_points.append(camera_T_point_4_pos)
                camera_T_points.append(camera_T_point_5_pos)
                camera_T_points.append(camera_T_point_6_pos)
                camera_T_points.append(camera_T_point_7_pos)
                
                results_cv2_points = []
                for i in range(len(camera_T_points)):
                    results_cv2_point = cv2.projectPoints(np.array(camera_T_points[i]),
                                                        rvecs,
                                                        tvecs,
                                                        np.array(_camera_intrinsic_matrix),
                                                        np.array(_dist_coeffs))
                    results_cv2_points.append(tuple([results_cv2_point[0][0][0][0]+x_par,results_cv2_point[0][0][0][1]+y_par]))
                points_OPTI = [results_cv2_points[0], 
                        results_cv2_points[1], 
                        results_cv2_points[2], 
                        results_cv2_points[3], 
                        results_cv2_points[4], 
                        results_cv2_points[5], 
                        results_cv2_points[6], 
                        results_cv2_points[7]]
                draw.draw_cube(points_OPTI, color)

            
            
        rgb_points_img = CvBridge().cv2_to_imgmsg(np.array(im)[..., ::-1], "bgr8")
        # rgb_points_img.header = camera_info.header
        
        self.pub_rgb_dope_points.publish(rgb_points_img)
        
class Draw(object):
    """Drawing helper class to visualize the neural network output"""

    def __init__(self, im):
        """
        :param im: The image to draw in.
        """
        self.draw = ImageDraw.Draw(im)

    def draw_line(self, point1, point2, line_color, line_width=5):
        """Draws line on image"""
        if point1 is not None and point2 is not None:
            # print(point1)
            # print(point2)
            self.draw.line([point1, point2], fill=line_color, width=line_width)
#            self.draw.draw_lines([point1, point2], fill=line_color, width=line_width)
            
    def draw_dot(self, point, point_color, point_radius):
        """Draws dot (filled circle) on image"""
        if point is not None:
            xy = [
                point[0] - point_radius,
                point[1] - point_radius,
                point[0] + point_radius,
                point[1] + point_radius
            ]
            self.draw.ellipse(xy,
                              fill=point_color,
                              outline=point_color
                              )

    def draw_cube(self, points, color=(255, 0, 0)):
        """
        Draws cube with a thick solid line across
        the front top edge and an X on the top face.
        """
        
        # draw front
        self.draw_line(points[0], points[1], color)
        self.draw_line(points[1], points[2], color)
        self.draw_line(points[3], points[2], color)
        self.draw_line(points[3], points[0], color)

        # draw back
        self.draw_line(points[4], points[5], color)
        self.draw_line(points[6], points[5], color)
        self.draw_line(points[6], points[7], color)
        self.draw_line(points[4], points[7], color)

        # draw sides
        self.draw_line(points[0], points[4], color)
        self.draw_line(points[7], points[3], color)
        self.draw_line(points[5], points[1], color)
        self.draw_line(points[2], points[6], color)

        # draw dots
        self.draw_dot(points[0], point_color=color, point_radius=4)
        self.draw_dot(points[1], point_color=color, point_radius=4)

        # draw x on the top
        self.draw_line(points[0], points[5], color)
        self.draw_line(points[1], points[4], color)
        
#cv_bridge = CvBridge()
#img = cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")

#draw = Draw()
#draw.draw_cube(points2d, self.draw_colors[m])

def rotation_4_4_to_transformation_4_4(rotation_4_4,pos):
    rotation_4_4[0][3] = pos[0]
    rotation_4_4[1][3] = pos[1]
    rotation_4_4[2][3] = pos[2]
    return rotation_4_4
def point_4_4_matrix(camera_T_obj_dope_4_4, point_3_3, point):
    point_4_4 = rotation_4_4_to_transformation_4_4(point_3_3,point)
    camera_T_point = np.dot(camera_T_obj_dope_4_4,point_4_4)
    point_pos = [camera_T_point[0][3],camera_T_point[1][3],camera_T_point[2][3]]
    return point_pos
def main():
    """Main routine to run DOPE"""

    # Initialize ROS node
    rospy.init_node('draw_box')
    listener = tf.TransformListener()
    while True:
        try:
            (trans,rot) = listener.lookupTransform('/RealSense', '/cracker', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    camera_T_obj_dope_pos = list(trans)
    camera_T_obj_dope_ori = list(rot)
    
    ros_listen = Ros_listener()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
# _camera_intrinsic_matrix = [[504.91994222,          0.0, 259.28746541],
#                             [         0.0, 503.71668498, 212.89422353],
#                             [         0.0,          0.0,          1.0]]
_camera_intrinsic_matrix = [[908.8558959960938,               0.0, 626.7174072265625],
                            [              0.0, 906.6900634765625, 383.2095947265625],
                            [              0.0,               0.0,               1.0]]

#_camera_intrinsic_matrix = [[605.903930664062,               0.0, 311.144958496094],
#                           [              0.0, 604.460021972656, 255.473068237305],
#                           [              0.0,               0.0,               1.0]]

# _camera_intrinsic_matrix = [[605.9547119140625,               0.0, 319.029052734375],
#                             [              0.0, 605.006591796875, 249.67617797851562],
#                             [              0.0,               0.0,               1.0]]


_dist_coeffs = [[0.], [0.], [0.], [0.]]
pybullet_robot_pos = [0.0, 0.0, 0.026]
pybullet_robot_ori = [0,0,0,1]
if __name__ == "__main__":
    # main()
    t_begin = time.time()
    DOPE_flag = True
    PBPF_flag = True
    Opti_flag = False
    optitrack_working_flag = False
    object_soup_flag = False
    object_cheezit_flag = True
    task_flag = "1"
    rospy.init_node('draw_box')
    listener = tf.TransformListener()
    alg_listener = Ros_listener_alg()
    OPTI_listener = Ros_listener_OPTI()
    ros_listener = Ros_listener()
