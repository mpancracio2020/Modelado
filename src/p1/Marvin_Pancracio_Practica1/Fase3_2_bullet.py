import pybullet as p
import pybullet_data
import argparse
import time
import math
import csv
import sys

VEL = 11.20
ZERO_VEL = 0.0
FORCE = 25.0
STEP_POS = 0.01
GOAL = 20
LATERAL_FRICTION = 0.93
SPINNING_FRICTION = 0.005
ROLLING_FRICTION = 0.003
FRONT_LEFT_WHEEL = 2
FRONT_RIGHT_WHEEL = 3
REAR_LEFT_WHEEL = 4
REAR_RIGHT_WHEEL = 5

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
floorId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
barrierId = p.loadURDF("barrier.urdf",[-2,17,0.1], startOrientation)
rampId = p.loadURDF("ramp.urdf",startPos, startOrientation)
goalId = p.loadURDF("goal.urdf",[0,20,0.1], startOrientation)

# Config dynamics...
for i in range(2,5):
    p.changeDynamics(robotId, i, lateralFriction = LATERAL_FRICTION)
    p.changeDynamics(robotId, i, spinningFriction = SPINNING_FRICTION)
    p.changeDynamics(robotId, i, rollingFriction = ROLLING_FRICTION)

def process_information(pos,init_time, file_csv): 
    
    # Method to get vel and pos information to fill csv file.

    time_passed = 0
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    linear_velocity, _,  = p.getBaseVelocity(robotId)
    current_velocity_y = linear_velocity[1] # get vel on y
    
    if  abs(position[0][1] - pos) >= 0.01: # By 0,01m take velocity and time to fill csv.

        pos = position[0][1]

        current_time = time.time() # Get time
        time_passed = abs(current_time - init_time)
        
        data = str(time_passed) + " " + str(pos) + " " +  str(current_velocity_y) + " " + str(VEL)+ " " + str(FORCE) +"\n"
        file_csv.write(data)

        if (pos >= GOAL): # end the record data and finish the execution.
            finish = 1
    return pos, finish

def follow_robot(): # Method to follow the robot.
    rPos, baseOrn = p.getBasePositionAndOrientation(robotId) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=3.0, cameraYaw = 0, cameraPitch=-40, cameraTargetPosition=rPos) # fix camera onto model

def main():
    # Init variables (position, time and program exec step)
    pos = 0.0
    init_time_clock = time.time()
    program_step = 0
    # Open/create csv file
    folder_name = "Fase3.2.csv"
    folder = open(folder_name, mode='w', newline='') 

    try:
        while True:
            if program_step == 0:            
                p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],p.VELOCITY_CONTROL, targetVelocities = [VEL,VEL,VEL,VEL], forces =[FORCE,FORCE,FORCE,FORCE])
                pos, program_step = process_information(pos,init_time_clock, folder) # update the position and get time.

            if program_step == 1:
                p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],p.VELOCITY_CONTROL, targetVelocities = [ZERO_VEL,ZERO_VEL,ZERO_VEL,ZERO_VEL])
                
    except KeyboardInterrupt:
        folder.close()
        pass
    p.disconnect()    

if __name__ == "__main__":
    main()