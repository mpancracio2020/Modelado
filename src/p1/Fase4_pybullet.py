import pybullet as p
import numpy as np
import pybullet_data
import argparse
import time
import math
import csv
import sys

VEL = 12.0
ZERO_VEL = 0.0
TORQUE = 100.0
ZERO_TORQUE = 0.0
STEP_POS = 0.01
LATERAL_FRICTION = 0.93
SPINNING_FRICTION = 0.005
ROLLING_FRICTION = 0.003
W = 3.0
D = 0.1
H = 0.1
M = 5.0
ixx = (M*(H**2 + D**2))/12
iyy = (M*(W**2 + H**2))/12
izz = (M*(W**2 + D**2))/12

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

robotOrientation = [0,0,math.pi/2] # set orientation robotId.
robotOrientationQ = p.getQuaternionFromEuler(robotOrientation)

planeId = p.loadURDF("plane.urdf")

robotId = p.loadURDF("husky/husky.urdf",[0, 0, 0], robotOrientationQ)
startPos = [0,10,0.1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

barrierId = p.loadURDF("barrier.urdf",[-3,17,0.1], startOrientation)
rampaId = p.loadURDF("rampa.urdf",startPos, startOrientation)
metaId = p.loadURDF("meta.urdf",[0,20,0.1], startOrientation)


target_velocity = 2.0  # Velocidad deseada en m/s

def process_information(pos,vel_input,motor_par):
   
    time_passed = 0
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    linear_velocity, _,  = p.getBaseVelocity(robotId)
    current_velocity_y = linear_velocity[1]
    
    if  abs(position[0][1] - pos) >= 0.01:

        pos = position[0][1]
        
        current_time = time.time()
        time_passed = abs(current_time - init_time)
        
        data = str(time_passed) + " " + str(pos) + " " +  str(vel_input) + " " + str(target_velocity)+ " " + str(motor_par) +"\n"

        folder.write(data)
        print(current_velocity_y)
        if (pos >= 20):
            finish = 1
            print(pos)

    return pos, finish
#----
# Configuración de parámetros

max_force = 4000  # Fuera máxima aplicable a las ruedas
Kp_inclination = 10.0  # Ganancia proporcional para la inclinación


#------
pos = 0.0
init_time = time.time()
folder_name = "Fase4.csv"
folder = open(folder_name, mode='w', newline='') 
program_step = 0
p.setVRCameraState([20,0,10])
p.changeDynamics(barrierId, 0, localInertiaDiagonal=[ixx,iyy,izz])

arg = input("Do you want to start? (y/n)... ")

if (arg == "y" or arg == "Y"):

    try:
        while True:
            
            p.setRealTimeSimulation(1)
            if program_step == 0:
        
                for i in range(2,5):
                    p.changeDynamics(robotId, i, lateralFriction = LATERAL_FRICTION)
                    p.changeDynamics(robotId, i, spinningFriction = SPINNING_FRICTION)
                    p.changeDynamics(robotId, i, rollingFriction = ROLLING_FRICTION)

                
                               
                _, orientation = p.getBasePositionAndOrientation(robotId)
                _, y, z_rotation = p.getEulerFromQuaternion(orientation)
                inclination_angle = math.degrees(y)

                
                # Calcular el par motor en función de la inclinación
                torque_factor = abs(Kp_inclination * inclination_angle)
                linear_velocity, _,  = p.getBaseVelocity(robotId)
                current_velocity_y = linear_velocity[1]
                # Calcular el error de velocidad en el eje y
                velocity_error_y = target_velocity - current_velocity_y

                
                if(abs(inclination_angle) >= 2):
                    vel_input = velocity_error_y*2
                    print("inclination: ", inclination_angle, " velocidad: ", current_velocity_y, " par: ", torque_factor*10000)
                    p.setJointMotorControlArray(robotId,[4,5],p.VELOCITY_CONTROL, targetVelocities = [velocity_error_y*2,velocity_error_y*2], forces=[torque_factor*100000,torque_factor*100000])
                    p.setJointMotorControlArray(robotId,[2,3],p.VELOCITY_CONTROL, targetVelocities = [velocity_error_y,velocity_error_y], forces=[torque_factor,torque_factor])
                
                else:
                    vel_input = velocity_error_y
                    print("inclination: ", inclination_angle, " velocidad: ", current_velocity_y, " par: ", torque_factor)
                    p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities = [velocity_error_y,velocity_error_y,velocity_error_y,velocity_error_y], forces=[torque_factor,torque_factor,torque_factor,torque_factor])
                
                #p.setJointMotorControlArray(robotId,[2,3,4,5],p.TORQUE_CONTROL, forces = [torque_factor,torque_factor,torque_factor,torque_factor])

               
                #p.stepSimulation()
                #time.sleep(1./240.)
                pos, program_step = process_information(pos, vel_input,torque_factor*100000) # update the position and get time.
                
            if program_step == 1:
                p.setJointMotorControlArray(robotId,[2,3,4,5],p.VELOCITY_CONTROL, targetVelocities = [ZERO_VEL,ZERO_VEL,ZERO_VEL,ZERO_VEL])

            
                
    except KeyboardInterrupt:
        folder.close()
        pass
        
p.disconnect()    