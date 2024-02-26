import pybullet as p
import pybullet_data
import argparse
import time
import math
import csv
import sys

VEL = 9.8
ZERO_VEL = 0.0
TORQUE = 10.0
ZERO_TORQUE = 0.0
STEP_POS = 0.01

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

robotOrientation = [0,0,math.pi/2] # set orientation husky.
robotOrientationQ = p.getQuaternionFromEuler(robotOrientation)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
startPos = [0,10,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

barrierId = p.loadURDF("barrier.urdf",[-1.8,17,0.1], startOrientation)
rampaId = p.loadURDF("rampa.urdf",startPos, startOrientation)
metaId = p.loadURDF("meta.urdf",[0,20,0.1], startOrientation)



def process_information(pos):
   
    time_passed = 0
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    velocity = p.getBaseVelocity(robotId)
    
    if  abs(position[0][1] - pos) >= 0.01:
        #print("y: " , pos)
        

        pos = position[0][1]
        wheels_vel = velocity[0][1]
        current_time = time.time()
        time_passed = abs(current_time - init_time)
        
        data = str(pos) + " " +str(time_passed) + " " +  str(velocity[0][1]) + " " + str(TORQUE) +"\n"

        folder.write(data)
        print(pos)
        if (pos >= 20):
            finish = 1
            print(pos)

    return pos, finish
     
pos = 0.0
init_time = time.time()
folder_name = "tests.csv"
folder = open(folder_name, mode='w', newline='') 
program_step = 0
p.setVRCameraState([20,0,10])


arg = input("Do you want to start? (y/n)... ")

if (arg == "y"):

    try:
        while True:
            
        
            if program_step == 0:
                p.setRealTimeSimulation(1)
                #p.stepSimulation()
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities=[VEL,VEL,VEL,VEL])
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.TORQUE_CONTROL, forces=[TORQUE,TORQUE,TORQUE,TORQUE])
                p.stepSimulation()
                #time.sleep(1./240.)
                pos, program_step = process_information(pos) # update the position and get time.

            if program_step == 1:
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities=[ZERO_VEL,ZERO_VEL,ZERO_VEL,ZERO_VEL])
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.TORQUE_CONTROL, forces=[ZERO_TORQUE,ZERO_TORQUE,ZERO_TORQUE,ZERO_TORQUE])

            
                
    except KeyboardInterrupt:
        folder.close()
        pass
        
p.disconnect()    