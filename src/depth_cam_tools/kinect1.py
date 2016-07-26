#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 3 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from rgbd_sensor_abstract import RGBDSensor
from sensor_msgs.msg import CameraInfo
import numpy as np
import os
import yaml
import rospy

class Kinect1(RGBDSensor):
    def __init__(self, camera_name, use_rect = True , use_ir = False, depth_topic = '', use_depth_registered = False, queue_size=1, compression=False):
        
        depth="depth"
        if use_depth_registered:
            depth = "depth_registered"
            
        rect=""
        if use_rect:
            rect="_rect"
        
        rgb_topic = camera_name+'/rgb/image'+rect+'_color'
        if use_ir:
            rgb_topic = ''
        
        if depth_topic == '':
            if use_depth_registered:
                depth_topic = camera_name+'/depth_registered/image_raw' 
            else:
                depth_topic = camera_name+'/'+depth+'/image'+rect+'_raw'
        
        ir_topic = camera_name+'/ir/image_raw'
        if use_rect:
            ir_topic = camera_name+'/ir/image_rect_ir'
        if not use_ir:
            ir_topic = ''
        
        depth_optical_frame = camera_name+'_depth_optical_frame'
            
        depth_camera_info = self.get_camera_info(camera_name, "depth")
        rgb_camera_info = self.get_camera_info(camera_name, "rgb")
            
        super(RGBDSensor, RGBDSensor(camera_name, depth_camera_info, rgb_camera_info, depth_optical_frame, rgb_topic, depth_topic, ir_topic, queue_size, compression))
        
        self.wait_until_ready()
    
    def get_camera_info(self, camera_name, img_name='depth'):
        camera_info = CameraInfo()
        file_url = ''
        try : 
            file_url = rospy.get_param(camera_name+'/driver/'+img_name+'_camera_info_url').replace('file://','')
        except Exception,e: print e
                
        if not os.path.exists(file_url):
            print 'ERROR: Could not read '+ camera_name+ ' '+img_name +'_camera_info'
            print '     Calibrate the sensor and try again !'
            exit(0)
            return
    
        print 'Loading camera '+img_name +'_camera_info for '+camera_name+' at:',file_url
        with open(file_url, 'r') as f:
            calib = yaml.safe_load(f.read())
            camera_info.K = np.matrix(calib["camera_matrix"]["data"])
            camera_info.D = np.array(calib["distortion_coefficients"]["data"])
            camera_info.R = np.matrix(calib["rectification_matrix"]["data"])
            camera_info.P = np.matrix(calib["projection_matrix"]["data"])
            camera_info.height = calib["image_height"]
            camera_info.width = calib["image_width"]
            print camera_info
        return camera_info
