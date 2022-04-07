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

ax_t = fig.add_subplot(gs_io[0])
ax_p = fig.add_subplot(gs_pid[0])
ax_i = fig.add_subplot(gs_pid[1])
ax_d = fig.add_subplot(gs_pid[2])
ax_m = fig.add_subplot(gs_io[1])
all_ax = [ax_t, ax_p, ax_i, ax_d, ax_m]

ax_t.get_xaxis().set_ticklabels([])
ax_p.get_xaxis().set_ticklabels([])
ax_i.get_xaxis().set_ticklabels([])

ax_t.set_title('Sensor Input')
ax_p.set_title('Proportional')
ax_i.set_title('Integral')
ax_d.set_title('Derivative')
ax_m.set_title('Motor Output')

# create data buffers
buffer_size = 100
x = [i for i in range(-buffer_size+1, 1)]
t_buff = [0 for _ in range(buffer_size)]
p_buff = [0 for _ in range(buffer_size)]
i_buff = [0 for _ in range(buffer_size)]
d_buff = [0 for _ in range(buffer_size)]
m_buff = [0 for _ in range(buffer_size)]

# plot data on graphs
t_line = ax_t.plot(x, t_buff, color='black')[0]
p_line = ax_p.plot(x, p_buff, color='red')[0]
i_line = ax_i.plot(x, i_buff, color='orange')[0]
d_line = ax_d.plot(x, d_buff, color='yellow')[0]
m_line = ax_m.plot(x, m_buff, color='black')[0]
ml_line = ax_m.plot(x, m_buff, color='blue')[0]
mr_line = ax_m.plot(x, m_buff, color='purple')[0]

for ax in all_ax:
    ax.grid()
    ax.set_xlim(x[0], x[-1])

fig.canvas.draw()

i = 1
while True: # main loop
    # get and store tilt data from robot
    tilt = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bot_id)[1])[1]*180/3.1415

    t_buff.append(tilt)
    t_buff.pop(0)

    # get user inputs from GUI
    turn = p.readUserDebugParameter(turn_input)
    traverse = -1*p.readUserDebugParameter(traverse_input)
    Kp = p.readUserDebugParameter(p_input)
    Ki = p.readUserDebugParameter(i_input)
    Kd = p.readUserDebugParameter(d_input)

    # calculate and store PID values
    P = t_buff[-1]/180.0
    I = sum([ti/180.0 for ti in t_buff])
    D = t_buff[-1]/180.0 - t_buff[-2]/180.0

    p_buff.append(P)
    i_buff.append(I)
    d_buff.append(D)
    p_buff.pop(0)
    i_buff.pop(0)
    d_buff.pop(0)

    # calculate and store control output based on PID values and coefficients
    ctrl = -1*(Kp*P + Ki*I + Kd*D)
    
    m_buff.append(ctrl)
    m_buff.pop(0)

    # set robot motor target speeds
    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL, targetVelocity=ctrl+turn+traverse)
    p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL, targetVelocity=ctrl-turn+traverse)

    # update graphs every 5th loop iteration
    if i >= 5:
        i = 1
        t_line.set_data(x, t_buff)
        p_line.set_data(x, p_buff)
        i_line.set_data(x, i_buff)
        d_line.set_data(x, d_buff)
        m_line.set_data(x, m_buff)
        ml_line.set_data(x, [m+turn+traverse for m in m_buff])
        mr_line.set_data(x, [m-turn+traverse for m in m_buff])

        for ax in all_ax:
            ax.relim()
            ax.autoscale_view(False,False,True)

        # ax_t.set_xlim(x[0], x[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()

    # step simulation, wait, then repeat
    p.stepSimulation()
    time.sleep(1./500.)
    i += 1
p.disconnect()
