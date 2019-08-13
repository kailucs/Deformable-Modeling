import logging
#logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option

from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
import math
import random

from robot_api.RobotController import UR5WithGripperController
import matplotlib.pyplot as plt
from scipy import signal
from copy import deepcopy

# TODO: temp set here
data_path = 'calibration_data/'
num_pic = 18
robot_host = '10.10.1.106'

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

    return 0

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

def picture():
    world = WorldModel()
    fn = "robot_model_data/ur5Blocks.xml" #correct UR5e model
    res = world.readFile(fn)
    robot = world.robot(0)
    ee_link=7
    link=robot.link(ee_link)
    ############### Load Calibration Joint Positions #######################3

    calibrationConfigs=[]
    dataFile=open(data_path+'calibration_joint_positions.txt','r')
    for line in dataFile:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        calibrationConfigs.append(l2)
    dataFile.close()

    print '---------------------model loaded -----------------------------' 

    serv=pyrs.Service()
    dev=serv.Device()
    #The settings below are for SR300 not F200
    dev.apply_ivcam_preset(0)
    try:  # set custom gain/exposure values to obtain good depth image
        custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 30.0),
                        (rs_option.RS_OPTION_R200_LR_GAIN, 100.0)]
        dev.set_device_options(*zip(*custom_options))
    except pyrs.RealsenseError:
        pass  # options are not available on all devices

    time.sleep(1)
    print '---------------------camera initiated -----------------------------'

    # control interface
    try:
        robotControlApi = UR5WithGripperController(host=robot_host,gripper=False)
        robotControlApi.start()
        time.sleep(2)
    except:
        print('robot lose connection.')
    print '---------------------robot started -----------------------------'

    ##########################################################
    ## Record some home configuration
    homeConfig=[-1.12599113833, -1.2611406849502682, 1.7447619654909734, 1.0871459070013214, 1.5707712083198737, 2.800998336141926, 0]#controller format
    constantVServo(robotControlApi,10.0,homeConfig,0.001)

    print '---------------------at home configuration -----------------------------'

    #EETransformFile=open('calibration_EE_transforms.txt',w)

    for i in range(num_pic):
        constantVServo(robotControlApi,2.0,calibrationConfigs[i],0.004)
        time.sleep(0.5)

        # For Color
        dev.wait_for_frames()
        cad = dev.cad
        cad = cv2.cvtColor(cad, cv2.COLOR_RGB2BGR)
        p=dev.points
        cv2.imwrite(data_path+'data/calbration_pic_'+str(i)+'.png',cad)
        cv2.imshow('color',cad)
        cv2.waitKey(500)
        data=open(data_path+'data/calibration_pt_'+str(i)+'.txt','w')
        for y in range(480):
            for x in range(640):
                data.write(str(p[y][x][0])+' '+str(p[y][x][1])+' '+str(p[y][x][2])+'\n')
        data.close()
        time.sleep(0.5)

        print('piont %d'%i)

    dev.stop()
    serv.stop()

    robotControlApi.stop()

if __name__ == "__main__":
    picture()