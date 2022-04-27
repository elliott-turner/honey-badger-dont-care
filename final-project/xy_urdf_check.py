# -*- coding: utf-8 -*-
"""
Created on Wed Apr 27 16:36:21 2022

@author: britt
"""

import pybullet as p 
import time 
import pybullet_data 

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-10) 
planeId = p.loadURDF("plane.urdf") 
start_pos = [0,0,1] 
start_orientation = p.getQuaternionFromEuler([0,0,0]) 
bot_id = p.loadURDF("xycartpole.urdf",start_pos, start_orientation) 
p.getJointInfo(bot_id, 0)
p.setJointMotorControl2(bot_id, 1, p.POSITION_CONTROL, force=0)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./250.)
p.disconnect()
