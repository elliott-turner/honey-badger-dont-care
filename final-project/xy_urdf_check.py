# -*- coding: utf-8 -*-
"""
Created on Wed Apr 27 16:36:21 2022

@author: britt
"""

import pybullet as p 
import time 
import pybullet_data 
import random

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-10) 
planeId = p.loadURDF("plane.urdf") 
start_pos = [0,0,1] 
start_orientation = p.getQuaternionFromEuler([0,0,0]) 
bot_id = p.loadURDF("xycartpole.urdf",start_pos, start_orientation) 

y_joint_id = 0
x_joint_id = 1
stick_x_joint_id = 2
stick_y_joint_id = 3
gimbal_link_id = 3
gimbal2Pole_Joint_id = 3

p.setJointMotorControl2(bot_id, stick_x_joint_id, p.POSITION_CONTROL, force=0)
p.setJointMotorControl2(bot_id, stick_y_joint_id, p.POSITION_CONTROL, force=0)

for i in range (10000):
    wind = random.uniform(-10, 10)

    p.applyExternalForce(bot_id, gimbal_link_id, (wind, 0, 0), (0, 0, 0), p.LINK_FRAME)
    p.stepSimulation()
    time.sleep(1./250.)
p.disconnect()
