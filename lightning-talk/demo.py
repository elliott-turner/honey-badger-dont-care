import pybullet as p
import time
import pybullet_data
from matplotlib import pyplot as plt
from matplotlib import gridspec

# setup simulation environment
physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0,0,1]
start_orientation = p.getQuaternionFromEuler([0,0,0])
bot_id = p.loadURDF("lightning-talk/bot.urdf",start_pos, start_orientation)
#set the center of mass frame (loadURDF sets base link frame) start_pos/Ornp.resetBasePositionAndOrientation(bot_id, start_pos, start_orientation)

# setup user inputs
turn_input = p.addUserDebugParameter('turn', -25, 25, 0)
traverse_input = p.addUserDebugParameter('traverse', -25, 25, 0)
p_input = p.addUserDebugParameter('P', 0, 100, 50)
i_input = p.addUserDebugParameter('I', 0, 10, 5)
d_input = p.addUserDebugParameter('D', 0 , 500, 250)

# setup graphs
plt.ion()
fig = plt.figure(figsize=(8,6))
gs = gridspec.GridSpec(1, 2, figure=fig)
gs_io = gs[0].subgridspec(2,1)
gs_pid = gs[1].subgridspec(3, 1)

subplots = [gs_io[0], gs_pid[0], gs_pid[1], gs_pid[2], gs_io[1]]
axs = [fig.add_subplot(subplot) for subplot in subplots]

axs[0].get_xaxis().set_ticklabels([])
axs[1].get_xaxis().set_ticklabels([])
axs[2].get_xaxis().set_ticklabels([])

titles = ['Sensor Input', 'Proportional', 'Integral', 'Derivative', 'Motor Output']
for ax, title in zip(axs, titles): ax.set_title(title)

# create data buffers
buffer_size = 100
x = [i for i in range(-buffer_size+1, 1)]
tbuff = [0 for _ in range(buffer_size)]
buffers = [
    [[0 for _ in range(buffer_size)]],
    [[0 for _ in range(buffer_size)]],
    [[0 for _ in range(buffer_size)]],
    [[0 for _ in range(buffer_size)]],
    [[0 for _ in range(buffer_size)], [0 for _ in range(buffer_size)], [0 for _ in range(buffer_size)]]
    ]

# plot data on graphs
colors = [['black'], ['red'], ['orange'], ['yellow'], ['black', 'blue', 'purple']]
lines = [[ax.plot(x, b, color=c)[0] for c,b in zip(color, buff)] for ax, buff, color in zip(axs, buffers, colors)]

ylims = [[-100, 100], [-1, 1], [-50, 50], [-0.1, 0.1], [-100, 100]]
for ax, ylim in zip(axs, ylims):
    ax.grid()
    ax.set_xlim(x[0], x[-1])
    ax.set_ylim(ylim[0], ylim[1])

fig.canvas.draw()
backgrounds = [fig.canvas.copy_from_bbox(ax.bbox) for ax in axs]

while True: # main loop
    # get and store tilt data from robot
    tilt = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bot_id)[1])[1]*180/3.1415

    buffers[0][0].append(tilt)
    buffers[0][0].pop(0)

    # get user inputs from GUI
    turn = p.readUserDebugParameter(turn_input)
    traverse = -1*p.readUserDebugParameter(traverse_input)
    Kp = p.readUserDebugParameter(p_input)
    Ki = p.readUserDebugParameter(i_input)
    Kd = p.readUserDebugParameter(d_input)

    # calculate and store PID values
    P = buffers[0][0][-1]/180.0
    I = sum([ti/180.0 for ti in buffers[0][0]])
    D = buffers[0][0][-1]/180.0 - buffers[0][0][-2]/180.0

    for v,j in zip([P,I,D], range(1,4)):
        buffers[j][0].append(v)
        buffers[j][0].pop(0)

    # calculate and store control output based on PID values and coefficients
    ctrl = -1*(Kp*P + Ki*I + Kd*D)

    ml = ctrl+turn+traverse
    mr = ctrl-turn+traverse
    
    buffers[4][0].append(ctrl)
    buffers[4][1].append(ml)
    buffers[4][2].append(mr)
    buffers[4][0].pop(0)
    buffers[4][1].pop(0)
    buffers[4][2].pop(0)

    # set robot motor target speeds
    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL, targetVelocity=ctrl)
    p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL, targetVelocity=ctrl)

    # update graphs
    for ax, line, buff, background in zip(axs, lines, buffers, backgrounds):
        fig.canvas.restore_region(background)
        for l, b in zip(line, buff): 
            l.set_ydata(b)
            ax.draw_artist(l)
        fig.canvas.blit(ax.bbox)

    # step simulation
    p.stepSimulation()
    # time.sleep(1./250.)
p.disconnect()
