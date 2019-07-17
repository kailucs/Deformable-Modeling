import logging
#logging.basicConfig(level=logging.INFO)
import time
import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option
from copy import deepcopy
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
from klampt import vis
from klampt import *
import math
import random
from robot_api.RobotController import UR5WithGripperController
import matplotlib.pyplot as plt
from scipy import signal
from copy import deepcopy   

### Constants and Measurements
tableHeight = 0.852
probeLength = 0.1
forceLimit = 5
dt=0.004
moveStep=0.002*dt   #2mm /s

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

def save_pcd(dev,name,number,transformCameraInWorld):
    dev.wait_for_frames()
    cad = deepcopy(dev.cad)  #size is 480 x 640 x 3
    c=deepcopy(dev.color)
    p = deepcopy(dev.points)
    print '----------------------------saving data--------------------------------------------'
    data=open(name+str(number)+'.xyzrgb','w')


    for i in range(480):
        for j in range(640):
            ptC=cad[i][j]
            pt=p[i][j]   
            if vectorops.norm(pt) > 0 and vectorops.norm_L1(ptC) > 0:
                ptW=se3.apply(transformCameraInWorld,pt)
                data.write(str(ptW[0])+' '+str(ptW[1])+' '+str(ptW[2])+' ')
                data.write(str(float(ptC[0])/255.0)+' '+str(float(ptC[1])/255.0)+' '+str(float(ptC[2])/255.0)+'\n')
    data.close()
    #print se3.apply(transformCameraInWorld,p[111][166]),se3.apply(transformCameraInWorld,p[111][477])
    #print se3.apply(transformCameraInWorld,p[380][166]),se3.apply(transformCameraInWorld,p[380][477])

def run_collection_PCD():
    #mode = "debugging"
    mode = "physical"

    ##################################### Start here ################################
    ## Initialize things 
    world = WorldModel()
    fn = "robo_model_data/ur5Blocks.xml" #correct UR5e model
    res = world.readFile(fn)
    robot = world.robot(0)
    ee_link=7
    link=robot.link(ee_link)

    ## Load calibrated camera transforms
    dataFile=open('calibrated_transforms/calibrated_camera_xform.txt','r')  ####### saved with  Klampt saver....... R is row major........
    for line in dataFile:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        transformCameraInEEtmp=(l2[0:9],l2[9:12]) #each transform is a tuple in klampt
    dataFile.close()
    transformCameraInEE=(so3.inv(transformCameraInEEtmp[0]),transformCameraInEEtmp[1])

    print '---------------------model loaded -----------------------------' 

    if mode == "physical":
        serv=pyrs.Service()
        dev=serv.Device()
        dev.apply_ivcam_preset(0)
        time.sleep(0.3)
        print '---------------------camera initiated -----------------------------'


    # control interface
    if mode == "physical":
        robotControlApi = UR5WithGripperController(host='10.10.1.106',gripper=False)
        robotControlApi.start()
        time.sleep(3)
        file = open("experiment_data/joint_positions.txt",'w')
        print '---------------------robot started -----------------------------'

    ## Record some home configuration
    homeConfig2=[-1.08105692024,-0.866032360067,1.67516698749,0.762010738422,1.57201803046,2.84750462606 , 0]#controller format
    if mode == "physical":
        print "------ moving home ---------"
        constantVServo(robotControlApi,5.0,homeConfig2,0.002)
        time.sleep(0.5)
        robotControlApi.stop()
        counter = 0
        for ele in homeConfig2:
            if counter <= 4:
                file.write(str(ele)+' ')
            else:
                file.write(str(ele)+'\n')
            counter = counter + 1

    ## update current position
    robot.setConfig(controller_2_klampt(robot,homeConfig2))
    EETransform=link.getTransform()
    if mode == "debugging":
        vis.add("world",world)
        #vis.add("ghost"+"_home",robot.getConfig())
    print '---------------------at home configuration -----------------------------'
    transformCameraInWorld=se3.mul(EETransform,transformCameraInEE)
    ############## Take point cloud picture ################
    # Take pcd data at different orientations

    if mode == "physical":
        save_pcd(dev,"experiment_data/objectScan_",0,transformCameraInWorld)

    objectCentroidLocal = [0.365,-0.098,0.0627]
    objectCentroidGlobal = se3.apply(EETransform,objectCentroidLocal)
    S=math.sin(15.0*3.14/180.0)
    C=math.cos(15.0*3.14/180.0)
    Z = [[-C,0,-S],[-1,0,0],[-C,0,S],[-1,0,0]] #rotate 20 degrees
    Y = [[0,-1,0],[0,-C,S],[0,-1,0],[0,-C,-S]]
    for i in range(4):
        local1 = objectCentroidLocal
        local2 = vectorops.add(objectCentroidLocal,[0,1,0])
        local3 = vectorops.add(objectCentroidLocal,[0,0,1])
        global1 = objectCentroidGlobal
        global2 = vectorops.add(objectCentroidGlobal,Y[i])
        global3 = vectorops.add(objectCentroidGlobal,Z[i])
        goal=ik.objective(link,local=[local1,local2,local3],world=[global1,global2,global3])
        res=ik.solve_nearby(goal,maxDeviation=1,tol=0.00001)
        if res:
            if mode == "debugging":
                vis.add("ghost"+str(i),robot.getConfig())
                vis.setColor("ghost"+str(i),0,1,0,0.5)
            elif mode == "physical":
                robotControlApi.start()
                time.sleep(1.0)
                constantVServo(robotControlApi,5.0,klampt_2_controller(robot.getConfig()),0.002)
                time.sleep(0.5)
                robotControlApi.stop()
                EETransform = link.getTransform()
                transformCameraInWorld = se3.mul(EETransform,transformCameraInEE)
                save_pcd(dev,"experiment_data/objectScan_",i+1,transformCameraInWorld)
                time.sleep(0.1)
                ## save the joint positions
                config = klampt_2_controller(robot.getConfig())
                counter = 0
                for ele in config:
                    if counter <= 4:
                        file.write(str(ele)+' ')
                    else:
                        file.write(str(ele)+'\n')
                    counter = counter + 1
        else:
            print "IK solver fail"

    if mode == "physical":  
        dev.stop()
        serv.stop()
        file.close()
    elif mode == "debugging":
        robot.setConfig(controller_2_klampt(robot,homeConfig2))
        vis.show()
        while vis.shown():
            time.sleep(1.0)
