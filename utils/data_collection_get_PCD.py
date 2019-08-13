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
'''
tableHeight = 0.852
probeLength = 0.1
forceLimit = 5
dt=0.004
moveStep=0.002*dt   #2mm /s
'''

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

def save_pcd_new(dev,name,number,transformCameraInWorld):
    st0 = time.time()
    dev.wait_for_frames()
    
    color_ori = deepcopy(dev.cad)
    position_ori = deepcopy(dev.points)

    color = np.reshape(np.array(color_ori),(640*480,3))/255.0
    pos_ori = np.reshape(np.array(position_ori),(640*480,3))
    xyzrgb_ori = np.hstack( (pos_ori,color) )

    # core! 
    #xyzrgb_ori = xyzrgb_ori[np.all(xyzrgb_ori != 0, axis = 1)]
    #xyzrgb_ori = xyzrgb_ori[xyzrgb_ori[:,0:3]!=np.array([0,0,0])]
    #xyzrgb_ori = xyzrgb_ori[xyzrgb_ori[:,3]!=0 or xyzrgb_ori[:,4]!=0 or xyzrgb_ori[:,5]!=0]
    num_points = xyzrgb_ori.shape[0]

    pos_ori = xyzrgb_ori[:,:3].T
    color = xyzrgb_ori[:,3:]
    print('[*]get xyzrgb of shape %s'%str(xyzrgb_ori.shape))

    pos_4item = np.vstack( ( pos_ori , np.ones((1,num_points)) ) )
    R_array = np.array(so3.matrix(transformCameraInWorld[0]))
    T_array = np.reshape(np.array(transformCameraInWorld[1]),(3,1))
    Trans_array = np.vstack( (np.hstack( (R_array,T_array)),np.array([0,0,0,1]) ) )
    pos_trans = np.dot(Trans_array,pos_4item)[:3,:]

    xyzrgb = np.hstack( (pos_trans.T,color) )

    new_xyzrgb_path = name+str(number)+'_new.npy'
    np.save(new_xyzrgb_path,xyzrgb)
    
    st3 = time.time()
    print('[*]saving pcd with time: %f s'%(st3-st0))

def save_pcd(dev,name,number,transformCameraInWorld):
    st0 = time.time()
    dev.wait_for_frames()
    
    color_ori = deepcopy(dev.cad)
    position_ori = deepcopy(dev.points)

    color = np.reshape(np.array(color_ori),(640*480,3))/255.0
    pos_ori = np.reshape(np.array(position_ori),(640*480,3))
    xyzrgb_ori = np.hstack( (pos_ori,color) )

    xyzrgb_ori = xyzrgb_ori[np.all(xyzrgb_ori != 0, axis = 1)]
    num_points = xyzrgb_ori.shape[0]

    pos_ori = xyzrgb_ori[:,:3].T
    color = xyzrgb_ori[:,3:]

    #print(pos_ori.shape)
    pos_4item = np.vstack( ( pos_ori , np.ones((1,num_points)) ) )
    R_array = np.array(so3.matrix(transformCameraInWorld[0]))
    T_array = np.reshape(np.array(transformCameraInWorld[1]),(3,1))
    Trans_array = np.vstack( (np.hstack( (R_array,T_array)),np.array([0,0,0,1]) ) )
    pos_trans = np.dot(Trans_array,pos_4item)[:3,:]

    xyzrgb = np.hstack( (pos_trans.T,color) )

    new_xyzrgb_path = name+str(number)+'_new.xyzrgb'
    st1 = time.time()
    np.savetxt(new_xyzrgb_path,xyzrgb,delimiter=' ')
    st2 = time.time()
    np.save('test.npy',xyzrgb)
    st3 = time.time()
    print('saving pcd with process time: %f s, save new pcd with time: %f s, npy with time: %f s'%(st1-st0,st2-st1,st3-st2))
    
    cad = color_ori  #size is 480 x 640 x 3
    p = position_ori
    print '----------------------------saving data--------------------------------------------'
    data=open(name+str(number)+'.xyzrgb','w')

    count = 0
    for i in range(480):
        for j in range(640):
            ptC=cad[i][j]
            pt=p[i][j]   
            if vectorops.norm(pt) > 0 and vectorops.norm_L1(ptC) > 0:
                ptW=se3.apply(transformCameraInWorld,pt)
                data.write(str(ptW[0])+' '+str(ptW[1])+' '+str(ptW[2])+' ')
                data.write(str(float(ptC[0])/255.0)+' '+str(float(ptC[1])/255.0)+' '+str(float(ptC[2])/255.0)+'\n')
                count = count+1
    data.close()
    et = time.time()

    print(count)
    print('saving pcd with time: %f s'%(et-st3))

def run_collection_PCD(config):

    mode = config.mode
    tableHeight = config.tableHeight
    probeLength = config.probeLength
    forceLimit = config.forceLimit
    dt= config.dt
    moveStep= config.moveStep
    debug_path = config.exp_path
    
    ##################################### Start here ################################
    ## Initialize things 
    world = WorldModel()
    fn = config.robot_model_path#correct UR5e model
    res = world.readFile(fn)
    robot = world.robot(0)
    ee_link=config.ee_link_number
    link=robot.link(ee_link)

    ## Load calibrated camera transforms
    dataFile=open(config.calibration_xform_path,'r')  ####### saved with  Klampt saver....... R is row major........
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
        robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
        robotControlApi.start()
        time.sleep(3)
        file1 = open(config.exp_path+'exp_'+str(config.exp_number)+"/joint_positions.txt",'w')
        print '---------------------robot started -----------------------------'

    ## Record some home configuration
    homeConfig2=config.home_config2#controller format
    if mode == "physical":
        print "------ moving home ---------"
        constantVServo(robotControlApi,5.0,homeConfig2,0.002)
        time.sleep(0.5)
        #robotControlApi.stop()
        counter = 0
        for ele in homeConfig2:
            if counter <= 4:
                file1.write(str(ele)+' ')
            else:
                file1.write(str(ele)+'\n')
            counter = counter + 1

    ## update current position
    robot.setConfig(controller_2_klampt(robot,homeConfig2))
    EETransform=link.getTransform()
    
    if mode == "debugging":
        vis.add("world",world)
    
    print '---------------------at home configuration -----------------------------'
    transformCameraInWorld=se3.mul(EETransform,transformCameraInEE)
       
    ############## Take point cloud picture ################
    # Take pcd data at different orientations

    if mode == "physical":
        save_pcd_new(dev,config.exp_path+'exp_'+str(config.exp_number)+"/objectScan_",0,transformCameraInWorld)
        #TODO:
    
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
                #robotControlApi.start()
                time.sleep(0.1)
                constantVServo(robotControlApi,2.0,klampt_2_controller(robot.getConfig()),0.004)
                time.sleep(0.1)
                #robotControlApi.stop()
                EETransform = link.getTransform()
                transformCameraInWorld = se3.mul(EETransform,transformCameraInEE)
                save_pcd_new(dev,config.exp_path+'exp_'+str(config.exp_number)+"/objectScan_",i+1,transformCameraInWorld)
                #TODO:
                time.sleep(0.1)
                ## save the joint positions
                config_robot = klampt_2_controller(robot.getConfig())
                counter = 0
                for ele in config_robot:
                    if counter <= 4:
                        file1.write(str(ele)+' ')
                    else:
                        file1.write(str(ele)+'\n')
                    counter = counter + 1
        else:
            print "IK solver fail"

    #get PCD done.
    robotControlApi.stop()

    if mode == "physical":  
        dev.stop()
        serv.stop()
        file1.close()
    
    elif mode == "debugging":
        robot.setConfig(controller_2_klampt(robot,homeConfig2))
        vis.show()
        print('[*] Debug: PCD collection done.')
        while vis.shown():
            time.sleep(1.0)
           
    print('[*] PCD collection done.')
