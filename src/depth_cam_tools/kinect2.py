#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 5 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from rgbd_sensor_abstract import RGBDSensor
from sensor_msgs.msg import CameraInfo
import numpy as np
import os
import yaml
import rospy

class Kinect2(RGBDSensor):
    def __init__(self, camera_name, use_rect = True, depth_topic = '' , queue_size=1, compression=False):
                    
        rect=""
        if use_rect:
            rect="_rect"
        
        rgb_topic = camera_name+'/sd/image_color'+rect
        if depth_topic == '':
            depth_topic = camera_name+'/sd/image_depth'+rect
        ir_topic = camera_name+'/sd/image_ir'+rect        
        
        depth_optical_frame = camera_name+'_ir_optical_frame'
            
        depth_camera_info = self.get_camera_info(camera_name, "ir")
        depth_camera_info.header.frame_id = depth_optical_frame
        rgb_camera_info = self.get_camera_info(camera_name, "color")
        rgb_camera_info.header.frame_id = depth_optical_frame
        
        super(RGBDSensor, RGBDSensor(camera_name, depth_camera_info, rgb_camera_info, depth_optical_frame, rgb_topic, depth_topic, ir_topic, queue_size, compression))
        
        self.wait_until_ready()
        
    def get_camera_info(self,camera_name,img_name='ir'):
        camera_info = CameraInfo()
        file_url = ''
        try : 
            file_url = rospy.get_param(camera_name+'_bridge/calib_path') + rospy.get_param(camera_name+'_bridge/sensor')+'/calib_'+img_name+'.yaml'
        except Exception,e: 
            print e

        if not os.path.exists(file_url):
            print 'ERROR: Could not read '+ camera_name+ ' '+img_name +'_camera_info'
            print '     Calibrate the sensor and try again !'
            exit(0)
            return
    
        print 'Loading camera '+img_name+' info at:',file_url
        with open(file_url, 'r') as f:
            stream = f.read()
            # HACK : remove new stuff addded by kinect2_calibration
            stream = stream.replace("%YAML:1.0","")
            stream = stream.replace("!!opencv-matrix","")
            calib = yaml.load(stream)
            camera_info.K = np.matrix(calib["cameraMatrix"]["data"])
            camera_info.D = np.array(calib["distortionCoefficients"]["data"])
            camera_info.R = np.matrix(calib["rotation"]["data"])
            camera_info.P = np.matrix(calib["projection"]["data"])
            # HACK : remove last line of projection matrix
            camera_info.P = np.delete(camera_info.P, [15,14,13,12])
            print camera_info
        return camera_info
