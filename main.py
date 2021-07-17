import os
import cv2
import random
import math
import time
import pybullet as p
from kuka import *
import numpy as np
import pybullet_data

# from camera import *


def stepSimulation():
    p.stepSimulation()
    time.sleep(1./240.)

def euclidean_dist(a, b):
    s = 0
    for el1, el2 in zip(a, b):
        s += abs(((el1*el1) - (el2*el2)))
    # s = max(s, 0)
    return math.sqrt(s)


physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

planeId = p.loadURDF("plane.urdf")

targetPos = [1, 0, 0.73]

doorId = p.loadURDF("../../Downloads/files/sphere.urdf", targetPos)

targetPos = [0.786, 0, 0.775]

kuka = Kuka()
kukaId = kuka.kukaUid
kukaEndEffectorIndex = 6

pos = kuka.getObservation()
# import pdb; pdb.set_trace()
threshold = 0.01
closeEnough = False
maxIter = 3000
iter = 0
kuka_pos = p.getLinkState(kukaId, kukaEndEffectorIndex)
end_effector_position = kuka_pos[0]
x_step = targetPos[0] - end_effector_position[0]
z_step = targetPos[1] - end_effector_position[1]
y_step = targetPos[2] - end_effector_position[2]

x_step = x_step/1000;
y_step = y_step/1000;
z_step = z_step/1000;

current_pos = [end_effector_position[0], end_effector_position[1], end_effector_position[2]]
j=0.1
while(j<0.8):
    jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, [j,0,0.775])

    for i in range(p.getNumJoints(kukaId)-2):
      p.resetJointState(kukaId, i, jointPoses[i])
    j=j+0.01
    p.stepSimulation()
    time.sleep(1/10)

# import pdb; pdb.set_trace()
    
# maxForce = 500
# targetVel = 0.2
# p.setJointMotorControl2(bodyUniqueId=kukaId, 
# 						jointIndex=7, 
# 						controlMode=p.VELOCITY_CONTROL,
# 						targetVelocity = targetVel,
# 						force = maxForce)
# stepSimulation()

while (not closeEnough and iter < maxIter):
	# while (not closeEnough and iter < maxIter):


    jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, targetPos)
    # if euclidean_dist(current_pos, targetPos) > 0.5:
    # 	current_pos[0] = current_pos[0] + x_step
    # 	current_pos[1] = current_pos[1] + y_step
    # 	current_pos[2] = current_pos[2] + z_step
    # else:
    # 	print('broken')
    # 	break
    for i in range(p.getNumJoints(kukaId)-2):
      p.resetJointState(kukaId, i, jointPoses[i])
    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    newPos = ls[4]
    diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    # closeEnough = (dist2 < threshold)
    iter = iter + 1
    maxForce = 200
    targetVel = 0.2
    p.setJointMotorControl2(bodyUniqueId=kukaId, 
							jointIndex=8, 
							controlMode=p.POSITION_CONTROL,
							targetPosition = -0.35,
							force = maxForce)
    if iter>200:
	    p.setJointMotorControl2(bodyUniqueId=kukaId, 
								jointIndex=7, 
								controlMode=p.VELOCITY_CONTROL,
								targetVelocity = -0.9,
								force = maxForce)
    stepSimulation()

import pdb; pdb.set_trace()
p.disconnect()