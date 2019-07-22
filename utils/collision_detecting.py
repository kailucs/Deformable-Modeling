#!/usr/bin/python
import sys
import time
import numpy as np
import time
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt import vis
from klampt.model import trajectory
from klampt.model import coordinates
from klampt.model import ik
from klampt.model import collide
import math
from robot_api.RobotController import UR5WithGripperController
import random
import matplotlib.pyplot as plt
from scipy import signal
from copy import deepcopy

import sys
import cv2
#sys.path.append('/home/trina2/GS/IV/IV_Code/')
#import IV_Ultra2Cam

## Convention: 
#User defined fuction _ connnect words and first word lowercase
#variable e.g. rightGripperLink
tableHeight = 0.77
safeHeight = 0.79
dt = 0.004
activeDoFs = [1,2,3,4,5,6]

# helper functions
def constantVServo(controller,servoTime,target,dt):
    currentTime=0.0
    goalConfig=deepcopy(target)
    currentConfig=controller.getConfig()
    difference=vectorops.sub(goalConfig,currentConfig)

    while currentTime < servoTime:
        setConfig=vectorops.madd(currentConfig,difference,currentTime/servoTime)
        controller.setConfig(setConfig)
        time.sleep(dt)
        currentTime=currentTime+dt
        print currentTime

def controller_2_klampt(robot,controllerQ):
	qOrig=robot.getConfig()
	q=[v for v in qOrig]
	for i in range(6):
		q[i+1]=controllerQ[i]
	return q

def klampt_2_controller(robotQ):

	temp=robotQ[1:7]
	temp.append(0)
	return temp

def get_instantaneous_trajectory(time):

	return (0,0.001) #1mm/s 

def move_Linear(robot,link,trans,rot,stepT,stepR,N,USLocation,Dev):
	Qs = []
	initialConfig = robot.getConfig()
	Qs.append(initialConfig)
	for i in range(N):
		oGlobal =  link.getWorldPosition(USLocation)# gripper frame origin
		#xG = link.getWorldPosition([1.0+gripperDistance,0,0])
		#yG = link.getWorldPosition([gripperDistance,1.0,0])
		xDG = link.getWorldDirection([1,0,0])
		yDG = link.getWorldDirection([0,1,0])
		currentTransform = link.getTransform()
		nextRotation = so3.mul(so3.from_axis_angle((rot,stepR))\
			,currentTransform[0])

		local1 = USLocation
		local2 = vectorops.add(local1,[1,0,0])
		local3 = vectorops.add(local1,[0,1,0])
		tmp = vectorops.mul(trans,stepT)
		global1 = vectorops.add(oGlobal,tmp)
		global2 = vectorops.add(global1,so3.apply(nextRotation,[1,0,0]))
		global3 = vectorops.add(global1,so3.apply(nextRotation,[0,1,0]))
		goal = ik.objective(link,local=[local1,local2,local3],world=[global1,global2,global3])
		if ik.solve_nearby(goal,maxDeviation=Dev,tol=0.0000001,activeDofs=activeDoFs):
			q = robot.getConfig()
			Qs.append(q)
			
		else:
			print "IK solver failure"
			return (0,0,False)
			

	robot.setConfig(initialConfig)
	return (Qs,q,True)

def check_collision_single(robot,collider,config):
	initialConfig = robot.getConfig()
	flag = False
	collisions = collider.robotSelfCollisions(robot)
	collisionsEnv = collider.robotTerrainCollisions(robot)
	#coliisionsObj = collider.robotObjectCollisions(robot)
	colCounter = 0
	for col in collisions:
		colCounter = colCounter + 1
	for col in collisionsEnv:
		colCounter = colCounter + 1

	if colCounter > 0:
		flag = True
		
	robot.setConfig(initialConfig)
	return flag 

def check_collision_linear(robot,collider,start,end,N):
	initialConfig = robot.getConfig()
	diff = vectorops.sub(end,start)
	flag = False
	for i in range(N):
		config = (vectorops.madd(start,diff,float(i)/float(N)))
		if check_collision_single(robot,collider,config):
			flag = True
			break
	if check_collision_single(robot,collider,end):
		flag = True
	robot.setConfig(initialConfig)
	return flag
def check_safety(robot,link,q):
	initialConfig = robot.initialConfig()

	robot.setConfig(q)
	p = link.getTransform()[1]

	if p[2] < 0.85:
		return False


	robot.setConfig(initialConfig)

	return True
