import pybullet as p
import pybullet_data
import argparse
import time
import math



physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

robotOrientation = [0,0,math.pi/2] # set orientation husky.
robotOrientationQ = p.getQuaternionFromEuler(robotOrientation)

planeId = p.loadURDF("plane_transparent.urdf")

#robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
startPos = [0,0,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

barrierId = p.loadURDF("barrier.urdf",[0,10,0.1], startOrientation)
rampaId = p.loadURDF("rampa.urdf",startPos, startOrientation)
frictionId = p.addUserDebugParameter("jointFriction", 0 ,40, 10)
torqueId = p.addUserDebugParameter("torque joint", 0 ,40, 5)

p.setRealTimeSimulation(1)


p.setRealTimeSimulation(1)

try:
    while True:
        frictionForce = p.readUserDebugParameter(frictionId)
        jointTorque = p.readUserDebugParameter(torqueId)
        # p.setJointMotorControl2(barrierId,0,p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        # p.setJointMotorControl2(barrierId,0,p.TORQUE_CONTROL, force=jointTorque)
        p.stepSimulation()
        time.sleep(1./24.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()    