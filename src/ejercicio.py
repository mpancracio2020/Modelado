import pybullet as p
import pybullet_data
import argparse
import time


parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="/ejercicio.urdf")
args = parser.parse_args()
urdf_path = args.urdf

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

robotId = p.loadURDF(urdf_path,startPos, startOrientation)
frictionId = p.addUserDebugParameter("jointFriction", 0 ,40, 10)
torqueId = p.addUserDebugParameter("torque joint", 0 ,40, 5)


#p.setRealTimeSimulation(1)

try:
    while True:
        frictionForce = p.readUserDebugParameter(frictionId)
        jointTorque = p.readUserDebugParameter(torqueId)
        p.setJointMotorControl2(robotId,1,p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        p.setJointMotorControl2(robotId,1,p.TORQUE_CONTROL, force=jointTorque)
        p.stepSimulation()
        time.sleep(1./24.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()    