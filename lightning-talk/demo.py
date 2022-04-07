import pybullet as p
import time
import pybullet_data
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0,0,1]
start_orientation = p.getQuaternionFromEuler([0,0,0])
bot_id = p.loadURDF("lightning-talk/bot.urdf",start_pos, start_orientation)
#set the center of mass frame (loadURDF sets base link frame) start_pos/Ornp.resetBasePositionAndOrientation(bot_id, start_pos, start_orientation)

turn_input = p.addUserDebugParameter('turn', -25, 25, 0)
traverse_input = p.addUserDebugParameter('traverse', -25, 25, 0)

p_input = p.addUserDebugParameter('P', 0, 50, 10)
i_input = p.addUserDebugParameter('I', 0, 50, 10)
d_input = p.addUserDebugParameter('D', 0 , 50, 10)

plt.ion()
fig = plt.figure()
fig.set_tight_layout(True)
ax = fig.add_subplot(1, 1, 1)
ax.yaxis.set_minor_locator(MultipleLocator(10))
ax.xaxis.set_minor_locator(MultipleLocator(5))
plt.title('Pitch vs. Simulation Steps')
plt.xlabel('Steps Since Simulation Start')
plt.ylabel('Pitch')
plt.grid()

buffer_size = 25
x = [i for i in range(-buffer_size+1, 1)]
y = [0 for _ in range(buffer_size)]

line, = ax.plot(x, y)
ax.set_ylim(-180, 180)
fig.canvas.draw()

Kp = 0.0
Ki = 0.0
Kd = 0.0
def pid():
    P = y[-1]/180.0
    I = sum([yi/180.0 for yi in y])
    D = y[-1]/180.0 - y[-2]/180.0
    return -1*(Kp*P + Ki*I + Kd*D)

i = 1
while True:
    pitch = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bot_id)[1])[1]*180/3.1415

    y.append(pitch)
    y.pop(0)

    Kp = p.readUserDebugParameter(p_input)
    Ki = p.readUserDebugParameter(i_input)
    Kd = p.readUserDebugParameter(d_input)
    ctrl = pid()

    turn = p.readUserDebugParameter(turn_input)
    traverse = p.readUserDebugParameter(traverse_input)

    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL, targetVelocity=ctrl+turn+traverse)
    p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL, targetVelocity=ctrl-turn+traverse)

    if i >= 5:
        i = 1
        line.set_data(x, y)
        ax.set_xlim(x[0], x[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()

    p.stepSimulation()
    time.sleep(1./500.)
    i += 1
p.disconnect()
