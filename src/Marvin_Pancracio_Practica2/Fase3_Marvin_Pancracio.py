import pybullet as p
import pybullet_data
import argparse
import time
import numpy as np
import matplotlib.pyplot as plt

# CONSTANTS:
LATERAL_FRICTION = 1
SPINNING_FRICTION = 1
ROLLING_FRICTION = 1
GOAL = 1.7
VEL = 10.0
ZERO = 0.0
FRONT_LEFT_WHEEL = 4
FRONT_RIGHT_WHEEL = 5
REAR_LEFT_WHEEL = 6
REAR_RIGHT_WHEEL = 7

# VELOCITY AND POSITION PARAMETERS
step = 0
gripPos = -2
jointTorque = 700
force = 500
vel = 1
vel_gains = 0.9
pos_gain = 0.03
tg_vel = 0.9
links_arm = [4,5,6,7,8,9,10,11,12,13,14]

forces_index = [force,force,force,force,force,force,force,force,force,force,force]
v = 0
vels = [v,v,v,v,v,v,v,v,v,v,v]


# POSITIONS AND ORIENTATIONS

pick_up = [4.1, 3.391320411329229e-05, 0.5999992980988114]
startPos = [0,0,2]
goalPos = [4, 0.0, 0.60]
cubePos = [4, 0, 0.025]
home_pos = [3.1109115058232466, 0.025383869144568152,   3.607527]
home_pos_pre_drop = [3.149997698951898, 0.0, 3.0]
pre_drop = [1.1800908229306726, 0.016919097772399163, 2.387248564224419]
pre_drop_2 = [1.1800908229306726, 0.016919097772399163,2.4]
home_pos_2 = [4.3109115058232466, 1.025383869144568152, 4]

startOrientation = p.getQuaternionFromEuler([0,0,-3.15])
orn = p.getQuaternionFromEuler([0,0,0])
orn_rot = p.getQuaternionFromEuler([0,3.14,0])
target_rotation = p.getQuaternionFromEuler([0, 0, 3.14])


parser = argparse.ArgumentParser(description="URDF viewer example")
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

# Load and config urdfs...
planeId = p.loadURDF("plane_transparent.urdf")
cubeId = p.loadURDF("Modelado/src/p1/cube.urdf",cubePos)
p.changeDynamics(cubeId, -1, lateralFriction=30)
p.changeDynamics(cubeId, -1, frictionAnchor=True)
p.changeDynamics(cubeId, -1, restitution=0)
p.changeDynamics(cubeId, -1, rollingFriction=0)
p.changeDynamics(cubeId, -1, spinningFriction=0)


robotId = p.loadURDF("urdf/u-bot.urdf",startPos)
p.changeDynamics(robotId, 13, lateralFriction=30)
p.changeDynamics(robotId, 13, frictionAnchor=True)
p.changeDynamics(robotId, 14, lateralFriction=30)
p.changeDynamics(robotId, 14, frictionAnchor=True)

for i in range(4,7):
    p.changeDynamics(robotId, i, lateralFriction = LATERAL_FRICTION)
    p.changeDynamics(robotId, i, spinningFriction = SPINNING_FRICTION)
    p.changeDynamics(robotId, i, rollingFriction = ROLLING_FRICTION)


def compare_pos(actual_pos, target_pos):
    # Return True if is at goal or close to it, and False if it is not.

    act_x = actual_pos[0]
    act_y = actual_pos[1]
    act_z = actual_pos[2]

    target_x = target_pos[0]
    target_y = target_pos[1]
    target_z = target_pos[2]

    diff_x = act_x - target_x
    diff_y = act_y - target_y
    diff_z = act_z - target_z
 
    if( abs(diff_x) <= 0.1 and abs(diff_y) <= 0.1 and abs(diff_z) <= 0.1):
        return True
    else:
        return False


def aproach_to_cube():
    # Put down the gripper .
    jointPose = p.calculateInverseKinematics(robotId,12,goalPos,orn_rot)

    invers_index = [jointPose[0],jointPose[1],jointPose[2],jointPose[3],jointPose[4],jointPose[5],jointPose[6],jointPose[7],jointPose[8],
                    jointPose[9],jointPose[10]]

    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                                targetVelocities=vels, 
                                positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                                velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                                )
    
def close_gripper():
    p.setJointMotorControlArray(robotId,[13,14],p.POSITION_CONTROL,targetPositions=[gripPos,gripPos], forces=[force,force],
                                targetVelocities = [tg_vel,tg_vel],
                                positionGains=[pos_gain,pos_gain], velocityGains=[vel_gains,vel_gains])


def checkpoint_1():
    home_Ik_Pose = p.calculateInverseKinematics(robotId,12,home_pos_pre_drop,orn_rot)
    p.setJointMotorControlArray(robotId,[13,14],p.POSITION_CONTROL,targetPositions=[gripPos,gripPos], forces=[force,force],
                                targetVelocities = [tg_vel,tg_vel],
                                positionGains=[pos_gain,pos_gain], velocityGains=[vel_gains,vel_gains])

    invers_index = [home_Ik_Pose[0],home_Ik_Pose[1],home_Ik_Pose[2],home_Ik_Pose[3],home_Ik_Pose[4],home_Ik_Pose[5],home_Ik_Pose[6],home_Ik_Pose[7],home_Ik_Pose[8],
                    home_Ik_Pose[9],home_Ik_Pose[10]]


    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            ) 
def checkpoint_2():
    pre_place_IK_Pose_1 = p.calculateInverseKinematics(robotId,8,home_pos_2,target_rotation)
    p.setJointMotorControlArray(robotId,[13,14],p.POSITION_CONTROL,targetPositions=[gripPos,gripPos], forces=[force,force],
                                targetVelocities = [tg_vel,tg_vel],
                                positionGains=[pos_gain,pos_gain], velocityGains=[vel_gains,vel_gains])
  
    invers_index = [pre_place_IK_Pose_1[0],pre_place_IK_Pose_1[1],pre_place_IK_Pose_1[2],pre_place_IK_Pose_1[3],pre_place_IK_Pose_1[4],pre_place_IK_Pose_1[5],pre_place_IK_Pose_1[6],pre_place_IK_Pose_1[7],pre_place_IK_Pose_1[8],
                    pre_place_IK_Pose_1[9],pre_place_IK_Pose_1[10]]

    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            ) 
    
def checkpoint_3():
    target_rotation = p.getQuaternionFromEuler([0, 0, 6.28])

    pre_place_IK_Pose_3 = p.calculateInverseKinematics(robotId,8,home_pos_2,target_rotation)
 
    invers_index = [pre_place_IK_Pose_3[0],pre_place_IK_Pose_3[1],pre_place_IK_Pose_3[2],pre_place_IK_Pose_3[3],pre_place_IK_Pose_3[4],pre_place_IK_Pose_3[5],pre_place_IK_Pose_3[6],pre_place_IK_Pose_3[7],pre_place_IK_Pose_3[8],
                    pre_place_IK_Pose_3[9],pre_place_IK_Pose_3[10]]


    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            )

def checkpoint_4():
    orn_T = p.getQuaternionFromEuler([0,0,0])

    pre_place_IK_Pose = p.calculateInverseKinematics(robotId,12,home_pos,orn_T)
  
    invers_index = [pre_place_IK_Pose[0],pre_place_IK_Pose[1],pre_place_IK_Pose[2],pre_place_IK_Pose[3],pre_place_IK_Pose[4],pre_place_IK_Pose[5],pre_place_IK_Pose[6],pre_place_IK_Pose[7],pre_place_IK_Pose[8],
                    pre_place_IK_Pose[9],pre_place_IK_Pose[10]]


    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            ) 
    


def checkpoint_5():
    target_rotation = p.getQuaternionFromEuler([0, 0, 0])

    pre_place_IK_Pose_3 = p.calculateInverseKinematics(robotId,8,home_pos_2,target_rotation)
  
    invers_index = [pre_place_IK_Pose_3[0],pre_place_IK_Pose_3[1],pre_place_IK_Pose_3[2],pre_place_IK_Pose_3[3],pre_place_IK_Pose_3[4],pre_place_IK_Pose_3[5],pre_place_IK_Pose_3[6],pre_place_IK_Pose_3[7],pre_place_IK_Pose_3[8],
                    pre_place_IK_Pose_3[9],pre_place_IK_Pose_3[10]]


    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            ) 
    

def checkpoint_6():
    target_rotation = p.getQuaternionFromEuler([0, 0, 0])

    pre_place_IK_Pose_3 = p.calculateInverseKinematics(robotId,12,pre_drop_2,target_rotation)

    invers_index = [pre_place_IK_Pose_3[0],pre_place_IK_Pose_3[1],pre_place_IK_Pose_3[2],pre_place_IK_Pose_3[3],pre_place_IK_Pose_3[4],pre_place_IK_Pose_3[5],pre_place_IK_Pose_3[6],pre_place_IK_Pose_3[7],pre_place_IK_Pose_3[8],
                    pre_place_IK_Pose_3[9],pre_place_IK_Pose_3[10]]

    p.setJointMotorControlArray(robotId,links_arm,p.POSITION_CONTROL,targetPositions=invers_index, forces=forces_index,  
                            targetVelocities=vels, 
                            positionGains=[pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain,pos_gain], 
                            velocityGains=[vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains,vel_gains]
                            )

def open_gripper():

    p.setJointMotorControlArray(robotId,[13,14],p.POSITION_CONTROL,targetPositions=[0.5,0.5], forces=[force,force],
                                targetVelocities = [tg_vel,tg_vel],
                                positionGains=[pos_gain,pos_gain], velocityGains=[vel_gains,vel_gains])
    
def go_ahead():
    finish = 0
    position = p.getBasePositionAndOrientation(robotId)
    pos = position[0][0] # position

    p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                p.VELOCITY_CONTROL, targetVelocities = [VEL,VEL,VEL,VEL])
    
    if (pos >= GOAL): # end the record data and finish the execution.
        finish = 1
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.VELOCITY_CONTROL, targetVelocities = [ZERO,ZERO,ZERO,ZERO])
        p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                    p.TORQUE_CONTROL, forces = [jointTorque,jointTorque,jointTorque,jointTorque])
    return finish

def process_foces(i_time,out_array,c):
    # Get forces info.
    if (abs(time.time()-i_time)) >= 0.01:
        jointState_5 = p.getJointState(robotId, 8)
        jointState_6 = p.getJointState(robotId, 9)
        jointState_7 = p.getJointState(robotId, 10)
        jointState_8 = p.getJointState(robotId, 11)
        jointState_9 = p.getJointState(robotId, 12)
        jointState_10 = p.getJointState(robotId, 13)
        jointState_11 = p.getJointState(robotId, 14)

        c = c + jointState_5[3] + jointState_6[3] + jointState_7[3] +jointState_8[3] +jointState_9[3] + jointState_10[3] + jointState_11[3]

        info_array = [time.time() - i_time , 7, c]
        out_array.append(info_array)
    return out_array, c

def  brakes():
    # Block the wheels
    p.setJointMotorControlArray(robotId,[FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, REAR_LEFT_WHEEL, REAR_RIGHT_WHEEL],
                                p.VELOCITY_CONTROL, targetVelocities = [ZERO,ZERO,ZERO,ZERO])

def main():
    a = 0
    arg = input("Do you want to start? (y/n)... ")
    step = 0
    cost = 0
    out_array = [ ]
    take_forces = 0
    g_total = 0
    if (arg == "y"):
        try:
            # Open/create csv file
            #folder_name = "p2.csv"
            #folder = open(folder_name, mode='w', newline='') 
            init_time = time.time()
            
            while True:
                p.stepSimulation()
                #print(p.getJointInfo(robotId,12)[0],p.getLinkState(robotId,12)[0])

                if step == 0:
                    # Move the robot close to the cube.
                    print("step: ", step)
                    step = go_ahead()

                if step == 1:
                    # Aproach the gripper to the cube.
                    print("step: ",step)
                    take_forces = 1
                    aproach_to_cube()
                
                    if( compare_pos(p.getLinkState(robotId,12)[0],pick_up)):
                        step = 2

                if step == 2:
                    # Grab the cube.
                    print("step: ",step)
                    brakes()
                    close_gripper()
                    a += 1
                    if a == 500:
                        step = 3
                        a=0

                if step == 3:
                    # Go to the first checkpoint.
                    brakes()
                    print("step: ",step)
                    checkpoint_1()
                    if compare_pos(p.getLinkState(robotId,12)[0],home_pos_pre_drop):
                        step = 4

                if step == 4:
                    # Go to the second checkpoint.
                    print("step: ", step)
                    brakes()
                    checkpoint_2()
                    if( compare_pos(p.getLinkState(robotId,12)[0],pre_drop)):
                        step = 5
                        open_gripper()

                if step == 5:
                    # Deposit the cube.
                    print("step: ", step)
                    open_gripper()
                    a+=1
                    if a == 200:
                        step = 6
                        a=0
                    
                if step == 6:
                    # Go to the third checkpoint.
                    print("step: ", step)
                    a += 1
                    checkpoint_5()
                    if a == 200:
                        step = 7
                        a =0
                
                if step == 7:
                    # Go home.
                    checkpoint_1()
                    step = 10
                    print("FINISHED !")
                if take_forces == 1:
                    out_array,cost = process_foces(init_time, out_array,cost)
                    g_total += cost
                time.sleep(1./200.)
            
        except KeyboardInterrupt:
            pass
    output_info = np.array(out_array)
    np.savetxt("Fase3_Marvin_Pancracio.csv", output_info, delimiter=",", fmt='%s')
    print(g_total)
    p.disconnect()    

if __name__ == "__main__":
    main()