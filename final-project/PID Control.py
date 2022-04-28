import pybullet as p
import time
import pybullet_data
import random
import math

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
start_pos = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
bot_id = p.loadURDF("xycartpole.urdf", start_pos, start_orientation)

y_joint_id = 0
x_joint_id = 1
stick_x_joint_id = 2
stick_y_joint_id = 3
linkx_id = 3
linky_id = 3
gimbal2Pole_Joint_id = 3
cart2Gimbal_Joint_id = 2

# print('link', p.getLinkState(bot_id, 4))

p.setJointMotorControl2(bot_id, stick_x_joint_id, p.POSITION_CONTROL, force=0)
p.setJointMotorControl2(bot_id, stick_y_joint_id, p.POSITION_CONTROL, force=0)

# set up params for PID control
Px = 0
Ix = 0
Dx = 0
Kpx = 0
Kix = 0
Kdx = 0

Py = 0
Iy = 0
Dy = 0
Kpy = 0
Kiy = 0
Kdy = 0

g = -10

testp = 80
testi = 10
testd = .05

prevUserV = [(0, 0)]

# setup user inputs

velocityx_input = p.addUserDebugParameter('Force on X-axis', -5, 5, 0)
velocityy_input = p.addUserDebugParameter('Force on Y-axis', -5, 5, 0)

px_input = p.addUserDebugParameter('Px', 0, 1000, testp)  # changed - these values were high
ix_input = p.addUserDebugParameter('Ix', 0, 1000, testi)  # changed - these values were high
dx_input = p.addUserDebugParameter('Dx', 0, 1000, testd)  # changed - these values were high

py_input = p.addUserDebugParameter('Py', 0, 1000, testp)  # changed - these values were high
iy_input = p.addUserDebugParameter('Iy', 0, 1000, testi)  # changed - these values were high
dy_input = p.addUserDebugParameter('Dy', 0, 1000, testd)  # changed - these values were high

tiltx_data = [0]
errorx_data = [0]

tilty_data = [0]
errory_data = [0]

step = 0

def controlV(velocity, g, step):
    a = velocity
    theta = math.degrees(math.asin(a/g))
    return theta

def movementStablize(velocity, g, step):
    a = velocity
    theta = math.degrees(math.asin(a / g))
    return -theta

while True:  # main loop
    # get and store tilt data from robot
    step += 1

    windStrength = 50
    windx = random.uniform(-windStrength, windStrength)
    windy = random.uniform(-windStrength, windStrength)

    #p.applyExternalForce(bot_id, linkx_id, (windx, 0, 0), (0, 0, 0), p.LINK_FRAME)
    #p.applyExternalForce(bot_id, linky_id, (0, windy, 0), (0, 0, 0), p.LINK_FRAME)

    # 0 to pi/2 to 0 to -pi/2
    tiltx = p.getEulerFromQuaternion(p.getLinkState(bot_id, linkx_id)[1])[1]
    tilty = p.getEulerFromQuaternion(p.getLinkState(bot_id, linky_id)[1])[0]

    tiltx_data.append(tiltx)
    tilty_data.append(tilty)

    errorx = tiltx_data[0] - tiltx_data[step]
    errorx_data.append(errorx)

    errory = tilty_data[0] - tilty_data[step]
    errory_data.append(errory)

    baseVx = p.getJointState(bot_id, 3)[1]
    baseVy = p.getJointState(bot_id, 3)[1]


    # get user inputs from GUI

    velocityx = p.readUserDebugParameter(velocityx_input)
    velocityy = p.readUserDebugParameter(velocityy_input)

    prevUserV.append((velocityx, velocityy))

    Kpx = p.readUserDebugParameter(px_input)
    Kix = p.readUserDebugParameter(ix_input)
    Kdx = p.readUserDebugParameter(dx_input)

    Kpy = p.readUserDebugParameter(py_input)
    Kiy = p.readUserDebugParameter(iy_input)
    Kdy = p.readUserDebugParameter(dy_input)

    # calculate and store PID values
    Px = errorx
    Ix += (errorx_data[step] - errorx_data[step - 1])/2 + errorx_data[step-1]
    Dx = (errorx_data[step] - errorx_data[step - 1])

    Py = errory
    Iy += (errory_data[step] - errory_data[step - 1]) / 2 + errory_data[step-1]
    Dy = (errory_data[step] - errory_data[step - 1])

    ctrlx = -(Kpx * Px + Kix * Ix + Kdx * Dx)
    ctrly = -(Kpy * Py + Kiy * Iy + Kdy * Dy)

    totalVelocityx = ctrlx + velocityx
    totalVelocityy = ctrly + velocityy

    if abs(velocityx) > 0:
        thetax = controlV(velocityx, g, step)
        tiltx_data[0] = thetax * ((math.pi / 2) / 90)

    if abs(velocityy) > 0:
        thetay = controlV(velocityy, g, step)
        tilty_data[0] = thetay * ((math.pi / 2) / 90)

    # if abs(velocityx) > 0:
    #     totalVelocityx = -velocityx
    #     if tiltx * (90/(math.pi/2)) == movementStablize(velocityx, g, step):
    #         totalVelocityx = ctrlx
    #
    # if abs(velocityy) > 0:
    #     totalVelocityy = -velocityy
    #     if tilty * (90/(math.pi/2)) == movementStablize(velocityy, g, step):
    #         totalVelocityy = ctrly


    # if abs(velocityx) > 0:
    #     thetay, velocityy = movementStablize(velocityy, g, step)
    #     tilty_data[0] = thetay * ((math.pi / 2) / 90)
    #     totalVelocityy = velocityy

    p.setJointMotorControl2(bot_id, x_joint_id, p.VELOCITY_CONTROL, targetVelocity=totalVelocityx)
    p.setJointMotorControl2(bot_id, y_joint_id, p.VELOCITY_CONTROL, targetVelocity=totalVelocityy)

    p.stepSimulation()
    time.sleep(1. / 250.)
p.disconnect()