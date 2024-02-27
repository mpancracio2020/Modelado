import pybullet as p
import pybullet_data
import argparse
import time
import math
import csv
import sys

VEL = 11.0
ZERO_VEL = 0.0
FORCE = 25.0
STEP_POS = 0.01
LATERAL_FRICTION = 0.93
SPINNING_FRICTION = 0.005
ROLLING_FRICTION = 0.003
W = 3.0
D = 0.1
H = 0.1
M = 5.0

# Inertial matrix:
ixx = (M*(H**2 + D**2))/12
iyy = (M*(W**2 + H**2))/12
izz = (M*(W**2 + D**2))/12

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)

# Initial pos and orientation config params: 

robotOrientation = [0,0,math.pi/2] # set orientation husky.
robotOrientationQ = p.getQuaternionFromEuler(robotOrientation)
startPos = [0,10,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

# Load urdfs...
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
barrierId = p.loadURDF("barrier.urdf",[-2,17,0.1], startOrientation)
rampaId = p.loadURDF("rampa.urdf",startPos, startOrientation)
metaId = p.loadURDF("meta.urdf",[0,20,0.1], startOrientation)

# Config dynamics...

p.changeDynamics(robotId, 1, lateralFriction = LATERAL_FRICTION)
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[ixx,iyy,izz])

for i in range(2,5):
    p.changeDynamics(robotId, i, lateralFriction = LATERAL_FRICTION)
    p.changeDynamics(robotId, i, spinningFriction = SPINNING_FRICTION)
    p.changeDynamics(robotId, i, rollingFriction = ROLLING_FRICTION)

def process_information(pos,init_time, file_csv): 
    
    # Method to get vel and pos information to fill csv file.

    time_passed = 0
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    velocity = p.getBaseVelocity(robotId)
    
    if  abs(position[0][1] - pos) >= 0.01: # By 0,01m take velocity and time to fill csv.

        pos = position[0][1]
        wheels_vel = velocity[0][1]
        current_time = time.time()
        time_passed = abs(current_time - init_time)
        
        data = str(time_passed) + " " + str(pos) + " " +  str(wheels_vel) + " " + str(VEL)+ " " + str(FORCE) +"\n"
        file_csv.write(data)

        if (pos >= 20): # end the record data and finish the execution.
            finish = 1
            
    return pos, finish

def main():
    # Init variables (position, time and program exec step)
    pos = 0.0
    init_time_clock = time.time()
    program_step = 0
    # Open/create csv file
    folder_name = "Fase3.csv"
    folder = open(folder_name, mode='w', newline='') 

    try:
        while True:
            if program_step == 0:            
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities = [VEL,VEL,VEL,VEL], forces =[FORCE,FORCE,FORCE,FORCE])
                pos, program_step = process_information(pos,init_time_clock, folder) # update the position and get time.

            if program_step == 1:
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities = [ZERO_VEL,ZERO_VEL,ZERO_VEL,ZERO_VEL])
                
    except KeyboardInterrupt:
        folder.close()
        pass
            
    p.disconnect()    

if __name__ == "__main__":
    main()