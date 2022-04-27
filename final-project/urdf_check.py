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
start_pos = [0,0,1] 
start_orientation = p.getQuaternionFromEuler([0,0,0]) 
bot_id = p.loadURDF("cartpole.urdf",start_pos, start_orientation) 
p.getJointInfo(bot_id, 0)
p.setJointMotorControl2(bot_id, 1, p.POSITION_CONTROL, force=0)

# set up params
P = 0
I = 0
D = 0
Kp = 0
Ki = 0
Kd = 0
last_error = 0

# setup user inputs
velocity_input = p.addUserDebugParameter('velocity', -25, 25, 0)
p_input = p.addUserDebugParameter('P', 0, 1, 1) # changed - these values were high 
i_input = p.addUserDebugParameter('I', 0, 2, 1) # changed - these values were high
d_input = p.addUserDebugParameter('D', 0 , 1, .05) # changed - these values were high
# for i in range (10000):   
#     p.stepSimulation()   
#     time.sleep(.005) 

tilt_data = []
while True: # main loop
    # get and store tilt data from robot
    # tilt = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bot_id)[1])[0]
    tilt = p.getBasePositionAndOrientation(bot_id)
    print(tilt)
    
    # initial_tilt = p.getEulerFromQuaternion(start_orientation)[1]

    tilt_data.append(tilt)
    # tilt_data.pop(0)
    
    # error = tilt - initial_tilt

    # # get user inputs from GUI
    # turn = p.readUserDebugParameter(turn_input)
    # traverse = -1*p.readUserDebugParameter(traverse_input)
    # Kp = p.readUserDebugParameter(p_input)
    # Ki = p.readUserDebugParameter(i_input)
    # Kd = p.readUserDebugParameter(d_input)

    # # calculate and store PID values
    # P = error
    # I = I + error
    # D = error - last_error
    
    # last_error = error

    # for v,j in zip([P,I,D], range(1,4)): # save PID data for plotting
    #     buffers[j][0].append(v)
    #     buffers[j][0].pop(0)

    # # calculate and store control output based on PID values and coefficients
    # ctrl = -1*(Kp*P + Ki*I + Kd*D)

    # ml = ctrl+turn+traverse
    # mr = ctrl-turn+traverse
    
    # for v,j in zip([ctrl, ml, mr], range(3)): # save motor control data for plotting
    #     buffers[4][j].append(v)
    #     buffers[4][j].pop(0)

    # # set robot motor target speeds
    # p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL, targetVelocity=ml)
    # p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL, targetVelocity=mr)

    # # update graphs
    # for ax, line, buff, background in zip(axs, lines, buffers, backgrounds):
    #     fig.canvas.restore_region(background)
    #     for l, b in zip(line, buff): 
    #         l.set_ydata(b)
    #         ax.draw_artist(l)
    #     fig.canvas.blit(ax.bbox)

    # step simulation
    p.stepSimulation()
    time.sleep(1./250.)
p.disconnect()


