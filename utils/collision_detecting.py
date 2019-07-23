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

def check_collision_single(robot,collider,config):
	initialConfig = robot.getConfig()
	flag = False
	robot.setConfig(config)
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
			robot.setConfig(config)
			break
	if check_collision_single(robot,collider,end):
		flag = True
	robot.setConfig(initialConfig)
	return flag
