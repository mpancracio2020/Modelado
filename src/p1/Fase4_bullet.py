import pybullet as p
import numpy as np
import pybullet_data
import argparse
import time
import math
import csv
import sys

VEL = 11.20
VEL_INC = 17.0
VEL_INC_2 = 20.0
VEL_INC_3 = 25.0    
VEL_INC_4 = 28.0
VEL_INC_5 = 30.0
VEL_INC_6 = 40.0
VEL_DEC = 8.20
ZERO_VEL = 0.0
FORCE = 25.0
STEP_POS = 0.01
GOAL = 20
TARGET_VELOCITY = 2.0
LATERAL_FRICTION = 0.93
SPINNING_FRICTION = 0.005
ROLLING_FRICTION = 0.003
INCLINATION_GAIN = 10.0
FRONT_LEFT_WHEEL = 2
FRONT_RIGHT_WHEEL = 3
REAR_LEFT_WHEEL = 4
REAR_RIGHT_WHEEL = 5
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
floorId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
barrierId = p.loadURDF("barrier.urdf",[-2.5,17,0.1], startOrientation)
rampId = p.loadURDF("ramp.urdf",startPos, startOrientation)
goalId = p.loadURDF("goal.urdf",[0,20,0.1], startOrientation)

# Config dynamics...

p.changeDynamics(robotId, 1, lateralFriction = LATERAL_FRICTION)
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[ixx,iyy,izz])

for i in range(2,5):
    p.changeDynamics(robotId, i, lateralFriction = LATERAL_FRICTION)
    p.changeDynamics(robotId, i, spinningFriction = SPINNING_FRICTION)
    p.changeDynamics(robotId, i, rollingFriction = ROLLING_FRICTION)

def process_information(pos,init_time,motor_par, file_csv):
   
    time_passed = 0
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    linear_velocity, _,  = p.getBaseVelocity(robotId)
    current_velocity_y = linear_velocity[1] # get vel on y
    
    if  abs(position[0][1] - pos) >= 0.01:
        pos = position[0][1]

        current_time = time.time() # Get time.
        time_passed = abs(current_time - init_time)
        
        data = str(time_passed) + " " + str(pos) + " " +  str(current_velocity_y) + " " + str(TARGET_VELOCITY)+ " " + str(motor_par) +"\n"
        file_csv.write(data)

        if (pos >= GOAL): # end the record data and finish the execution.
            finish = 1 

    return pos, finish
def follow_robot():
    rPos, baseOrn = p.getBasePositionAndOrientation(robotId) # Get model position
    p.resetDebugVisualizerCamera( cameraDistance=3.0, cameraYaw = 0, cameraPitch=-40, cameraTargetPosition=rPos) # fix camera onto model

def control(inclination_angle,force_factor):
    # Here compute the necessary force to try to keep 2m/s velocity.
    
    if(inclination_angle <= -20):
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL]
                                    ,p.VELOCITY_CONTROL, targetVelocities = [VEL_INC,VEL_INC,VEL_INC,VEL_INC], 
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    elif( inclination_angle <=  -22):
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL_INC_2,VEL_INC_2,VEL_INC_2,VEL_INC_2], 
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    elif( inclination_angle <=  -25):  
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL_INC_3,VEL_INC_3,VEL_INC_3,VEL_INC_3],
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    elif( inclination_angle <=  -30):    
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL_INC_4,VEL_INC_4,VEL_INC_4,VEL_INC_4],
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    elif( inclination_angle <=  -32):    
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL_INC_5,VEL_INC_5,VEL_INC_5,VEL_INC_5],
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    elif( inclination_angle <=  -35):   
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL_INC_6,VEL_INC_6,VEL_INC_6,VEL_INC_6],
                                    forces=[force_factor,force_factor,force_factor,force_factor])
    else:
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [VEL,VEL,VEL,VEL])


def main():
    # Init variables (position, time and program exec step)
    pos = 0.0
    init_time_clock = time.time()
    program_step = 0
    # Open/create csv file
    folder_name = "Fase4.csv"
    folder = open(folder_name, mode='w', newline='') 
   
    try:
        while True:

            #follow_robot() # use it to follow the robot
            if program_step == 0:
            
                _, orientation = p.getBasePositionAndOrientation(robotId)
                _, y, _ = p.getEulerFromQuaternion(orientation)
                inclination_angle_y = math.degrees(y)

                # Compute force due to inclination
                force_factor = abs(INCLINATION_GAIN * inclination_angle_y)
                
                control(inclination_angle_y,force_factor)
                pos, program_step = process_information(pos,init_time_clock,force_factor,folder) # update the position and get time.
               
            if program_step == 1:
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities = [ZERO_VEL,ZERO_VEL,ZERO_VEL,ZERO_VEL])
                
    except KeyboardInterrupt:
        folder.close()
        pass
        
    p.disconnect()    

if __name__ == "__main__":
    main()