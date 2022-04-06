import pybullet as p
import time
import pybullet_data
physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0,0,1]
start_orientation = p.getQuaternionFromEuler([0,0,0])
bot_id = p.loadURDF("lightning-talk/bot.urdf",start_pos, start_orientation)
#set the center of mass frame (loadURDF sets base link frame) start_pos/Ornp.resetBasePositionAndOrientation(bot_id, start_pos, start_orientation)

for joint_num in range(p.getNumJoints(bot_id)):
    print(p.getJointInfo(bot_id, joint_num))

left_wheel_input = p.addUserDebugParameter('left wheel', -25, 25, 0)
right_wheel_input = p.addUserDebugParameter('right wheel', -25, 25, 0)

for i in range (10000):
    p.setJointMotorControl2(bot_id, 0, p.VELOCITY_CONTROL,
        targetVelocity=p.readUserDebugParameter(left_wheel_input))
    p.setJointMotorControl2(bot_id, 1, p.VELOCITY_CONTROL,
        targetVelocity=p.readUserDebugParameter(right_wheel_input))
    p.stepSimulation()
    time.sleep(1./500.)
p.disconnect()
