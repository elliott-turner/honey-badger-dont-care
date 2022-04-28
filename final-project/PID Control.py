import pybullet as p
import time
import pybullet_data
import random

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

testp = 50
testi = 30
testd = 5

# setup user inputs
velocityx_input = p.addUserDebugParameter('velocityX', -5, 5, 0)
velocityy_input = p.addUserDebugParameter('velocityY', -5, 5, 0)

px_input = p.addUserDebugParameter('Px', 0, 200, testp)  # changed - these values were high
ix_input = p.addUserDebugParameter('Ix', 0, 150, testi)  # changed - these values were high
dx_input = p.addUserDebugParameter('Dx', 0, 10, testd)  # changed - these values were high

py_input = p.addUserDebugParameter('Py', 0, 200, testp)  # changed - these values were high
iy_input = p.addUserDebugParameter('Iy', 0, 150, testi)  # changed - these values were high
dy_input = p.addUserDebugParameter('Dy', 0, 10, testd)  # changed - these values were high

tiltx_data = [0]
errorx_data = [0]

tilty_data = [0]
errory_data = [0]

step = 0

while True:  # main loop
    # get and store tilt data from robot
    step += 1

    windStrength = 25
    windx = random.uniform(-windStrength, windStrength)
    windy = random.uniform(-windStrength, windStrength)

    p.applyExternalForce(bot_id, linkx_id, (windx, 0, 0), (0, 0, 0), p.LINK_FRAME)
    p.applyExternalForce(bot_id, linky_id, (0, windy, 0), (0, 0, 0), p.LINK_FRAME)

    # 0 to pi/2 to 0 to -pi/2
    tiltx = p.getEulerFromQuaternion(p.getLinkState(bot_id, linkx_id)[1])[1]
    tilty = p.getEulerFromQuaternion(p.getLinkState(bot_id, linky_id)[1])[0]

    tiltx_data.append(tiltx)
    tilty_data.append(tilty)

    errorx = tiltx_data[0] - tiltx_data[step]
    errorx_data.append(errorx)

    errory = tilty_data[0] - tilty_data[step]
    errory_data.append(errory)

    baseVx = p.getBaseVelocity(1)[0][0]
    baseVy = p.getBaseVelocity(1)[0][1]


    # get user inputs from GUI

    velocityx = p.readUserDebugParameter(velocityx_input)
    velocityy = p.readUserDebugParameter(velocityy_input)

    Kpx = p.readUserDebugParameter(px_input)
    Kix = p.readUserDebugParameter(ix_input)
    Kdx = p.readUserDebugParameter(dx_input)

    Kpy = p.readUserDebugParameter(py_input)
    Kiy = p.readUserDebugParameter(iy_input)
    Kdy = p.readUserDebugParameter(dy_input)

    # calculate and store PID values
    Px = errorx
    Ix += (errorx_data[step] - errorx_data[step - 1])/2
    Dx = -1 * (tiltx_data[step] - tiltx_data[step - 1])

    Py = errory
    Iy += (errory_data[step] - errory_data[step - 1]) / 2
    Dy = -1 * (tilty_data[step] - tilty_data[step - 1])

    ctrlx = -(Kpx * Px + Kix * Ix + Kdx * Dx)
    ctrly = -(Kpy * Py + Kiy * Iy + Kdy * Dy)

    totalVelocityx = ctrlx + velocityx
    totalVelocityy = ctrly + velocityy

    print(totalVelocityx, totalVelocityy, step)

    p.setJointMotorControl2(bot_id, x_joint_id, p.VELOCITY_CONTROL, targetVelocity=totalVelocityx)
    p.setJointMotorControl2(bot_id, y_joint_id, p.VELOCITY_CONTROL, targetVelocity=totalVelocityy)

    p.stepSimulation()
    time.sleep(1. / 250.)
p.disconnect()