#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 3 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from rgbd_sensor_abstract import RGBDSensor

class XtionProLive(RGBDSensor):
    def __init__(self, camera_name, use_rect = True , use_depth_registered = False, queue_size=1, compression=False):
        
        depth="depth"
        if use_depth_registered:
            depth = "depth_registered"
            
        rect=""
        if use_rect:
            rect="_rect"
        
        rgb_topic = camera_name+'/rgb/image'+rect+'_color'
        
        depth_topic = camera_name+'/'+depth+'/image'+rect+'_raw'
        if use_depth_registered:
            depth_topic = camera_name+'/depth_registered/image_raw' 
        
        ir_topic = camera_name+'/ir/image_raw'
        if use_rect:
            ir_topic = camera_name+'/ir/image_rect_ir'   
        
        ir_topic = ''
            
        super(RGBDSensor, RGBDSensor(camera_name, rgb_topic, depth_topic, ir_topic, use_depth_registered, queue_size, compression))
        
        self.wait_until_ready()
