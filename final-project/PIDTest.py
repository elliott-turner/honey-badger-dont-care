import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
start_pos = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
bot_id = p.loadURDF("cartpole.urdf", start_pos, start_orientation)
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
velocity_input = p.addUserDebugParameter('velocity', -5, 5, 0)
p_input = p.addUserDebugParameter('P', 0, 100, 100)  # changed - these values were high
i_input = p.addUserDebugParameter('I', 0, 100, 100)  # changed - these values were high
d_input = p.addUserDebugParameter('D', 0, 1, .05)  # changed - these values were high

tilt_data = [0]
error_data = [0]
step = 0

g = -10
baseV = [0]
baseA = [0]

# calculate and store PID values
def PID(error, error_data, tilt_data, P, I, D):

    P = error

    I += (error_data[step] - error_data[step - 1]) / 2

    D = -1 * (tilt_data[step] - tilt_data[step - 1])

    ctrl = (Kp * P + Ki * I + Kd * D)

    return ctrl

def velocityControl():


while True:  # main loop
    # get and store tilt data from robot
    step += 1

    # 0 to pi/2 to 0 to -pi/2
    tilt = p.getEulerFromQuaternion(p.getLinkState(bot_id, 1)[1])[1]

    tilt_data.append(tilt)

    error = tilt_data[0] - tilt_data[step]
    error_data.append(error)

    # get user inputs from GUI
    velocity = p.readUserDebugParameter(velocity_input)
    Kp = p.readUserDebugParameter(p_input)
    Ki = p.readUserDebugParameter(i_input)
    Kd = p.readUserDebugParameter(d_input)

    baseV.append(p.getBaseVelocity(bot_id)[0][0])
    baseA.append(baseV[step] - baseV[step])

    cart_vel = PID(error, error_data, tilt_data, P, I, D) + velocity

    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL, targetVelocity=-cart_vel)

    # step simulation
    p.stepSimulation()
    time.sleep(1. / 250.)
p.disconnect()