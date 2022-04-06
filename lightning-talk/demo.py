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

left_wheel_input = p.addUserDebugParameter('left wheel', -25, 25, 0)
right_wheel_input = p.addUserDebugParameter('right wheel', -25, 25, 0)

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
x = [-2, -1]
y = [0, 0]

line, = ax.plot(x, y)
ax.set_ylim(-180, 180)

fig.canvas.draw()

for i in range (10000):
    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL,
        targetVelocity=p.readUserDebugParameter(left_wheel_input))
    p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL,
        targetVelocity=p.readUserDebugParameter(right_wheel_input))

    pitch = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bot_id)[1])[1]*180/3.1415

    if i%5 == 0:
        x.append(i)
        y.append(pitch)
        if len(x) > buffer_size:
            x.pop(0)
            y.pop(0)

        line.set_data(x, y)
        ax.set_xlim(x[0], x[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()

    p.stepSimulation()
    time.sleep(1./500.)
p.disconnect()
