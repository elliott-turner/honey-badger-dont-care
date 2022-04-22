# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 19:37:02 2022

@author: britt
"""

import pybullet as p 
import time 
import pybullet_data 

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally 
p.setGravity(0,0,-10) 
planeId = p.loadURDF("plane.urdf") 
cubeStartPos = [0,0,1] 
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0]) 
boxId = p.loadURDF("inverse_pendulum.urdf",cubeStartPos, cubeStartOrientation) 
for i in range (10000):   
    p.stepSimulation()   
    time.sleep(.005) 

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId) 
print(cubePos,cubeOrn) 
p.disconnect()
  

