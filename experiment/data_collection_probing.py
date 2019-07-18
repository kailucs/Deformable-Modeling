#import logging
#logging.basicConfig(level=logging.INFO)
import time
import numpy as np
import cv2
from copy import deepcopy
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
from klampt import vis
import math
import random
from robot_api.RobotController import UR5WithGripperController
import matplotlib.pyplot as plt
from scipy import signal

#emulator
#from RobotControllerEmulator import kill_controller_threads,UR5WithGripperController

'''
#CONTROLLER = 'simulation'
CONTROLLER = 'physical'
#CONTROLLER = 'debugging'

### Constants and Measurements
tableHeight = 0.87
probeLength = 0.09
forceLimit = 2.0
dt=0.004  #250Hz
moveStep=0.002*dt   #2mm /s
shortServoTime=1.5
longServoTime=3
IKErrorTolerence=4
maxDev=1.2
EEZLimit=0.956
'''

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
        #print currentTime

    return 0

def run_poking(config):
	##################################### Start here ################################
	## Initialize things 
	world = WorldModel()
	res = world.readFile(config.robot_model_path)
	robot = world.robot(0)
	ee_link=config.ee_link_number
	link=robot.link(ee_link)
	CONTROLLER = config.mode

	### Constants and Measurements
	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt=config.dt  #250Hz
	moveStep=0.002*dt   #2mm /s

	shortServoTime=config.shortServoTime
	longServoTime=config.longServoTime+2 #TODO: set in config?
	IKErrorTolerence=config.IKErrorTolerence
	maxDev=config.maxDev
	EEZLimit=config.EEZLimit

	print '---------------------model loaded -----------------------------' 

	########################## Read In the pcd ######################################
	points=[]
	normals=[]
	dataFile=open(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt','r')
	for line in dataFile:
		line=line.rstrip()
		l=[num for num in line.split(' ')]
		l2=[float(num) for num in l]
		points.append(l2[0:3])
		normals.append(l2[6:9])
	dataFile.close()

	print '---------------------pcd loaded -----------------------------' 

	# TODO: no use
	if CONTROLLER == 'simulation':
		controllerWorld = world.copy()

	# control interface
	elif CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
	print '---------------------robot started -----------------------------'

	## Record some home configuration
	homeConfig2=config.home_config2#controller format
	intermediateConfig = config.intermediateConfig
	homeConfig2 = intermediateConfig

	if CONTROLLER == "physical":
		constantVServo(robotControlApi,4,homeConfig2,dt)#controller format

	robot.setConfig(controller_2_klampt(robot,homeConfig2))
	EETransform=link.getTransform()
	print '---------------------at home configuration -----------------------------'



	if CONTROLLER == 'debugging':
		vis.add("world",world)
		differences=[]
		print('[*]Debug: Poking process start')
		#for i in range(len(points)):
		# TODO:
		point_list = input('There are %d poking point, input a list:'%len(points))
		if point_list == len(points):
			point_list == range(len(points))
		
		for i in point_list:
			print('point %d'%i)
			print points[i],normals[i]
			robotCurrentConfig=homeConfig2
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))
			#print 'approach vector',approachVector
			#EEgoalPosition=vectorops.add(goalPosition,vectorops.mul(approachVector,-probeLength))
			## perform IK
			pt1=goalPosition
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			localZUnitV=vectorops.cross([0,1,0],approachVector)
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
				#print robotCurrentConfig
				#print klampt_2_controller(robot.getConfig())
				#print 'EE position',EEZPos
				print 'difference', diff	
				differences.append(diff)

				vis.add("ghost"+str(i),robot.getConfig())
				vis.setColor("ghost"+str(i),0,1,0,0.5)	

				if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
					pass
				else:
					print "IK too far away"
					break
			else:
				print "IK failture"
				break


			### now start colecting data..
			#wrench = robotControlApi.getWrench()
			#xForce = wrench[2]
			travel = 0.0
			#forceHistory=[xForce]
			#displacementHistory=[travel]
			stepVector = vectorops.mul(approachVector,moveStep)
			while travel<0.0001:
				robotCurrentConfig=klampt_2_controller(robot.getConfig())
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
					#print robotCurrentConfig
					#print klampt_2_controller(robot.getConfig())
					#print 'EE position',EEZPos
					#print 'difference', diff	
					if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
						pass
					else:
						print "IK too far away"
						break
				else:
					print "IK failture"
					break
				#time.sleep(dt)
				#xForce = robotControlApi.getWrench()[2]
				travel = travel + moveStep
				#forceHistory.append(xForce)
				#displacementHistory.append(travel)

			### move the probe away
			robotCurrentConfig=klampt_2_controller(robot.getConfig())
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
				#print robotCurrentConfig
				#print klampt_2_controller(robot.getConfig())
				#print 'EE position',EEZPos
				#print 'difference', diff	
				if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
					pass
				else:
					print "IK too far away"
					break
			else:
				print "IK failture"
				break

			### move back to intermediate config
			robot.setConfig(controller_2_klampt(robot,intermediateConfig))
		
		print('[*]Debug: Poking process done, with max difference:%f'%max(differences))

	elif CONTROLLER == 'physical':
		######################################## Ready to Take Measurements ################################################
		#TODO: arrange the point execute order
		exe_number = input('There are %d poking point, input executing number:'%len(points))
		point_list == random.sample(range(len(points)),exe_number)
		#K = np.arange(start = 132, stop = len(points)-1)
		#print K
		for i in point_list:	
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			#calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))
			#sapproachVector = [0,0,-1]
			#### Make sure no contact, backup 0.01m
			pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,-0.01))
			#pt1=goalPosition
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			localZUnitV=vectorops.cross([0,1,0],approachVector)
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
				#print robotCurrentConfig
				#print klampt_2_controller(robot.getConfig())
				#print 'EE position',EEZPos
				#print 'difference', diff	
				if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
					constantVServo(robotControlApi,longServoTime,klampt_2_controller(robot.getConfig()),dt)
				else:
					print "IK too far away"
					break
			else:
				print "IK failture"
				break

			#print klampt_2_controller(robot.getConfig())
			time.sleep(0.2)

			## Zero the sensor before straight line push
			#
			#
			# Note that the force is recorded in the global frame..
			# And the global frame has x and y axis flipped w.r.t the URDF....

			counter = 0.0
			totalF = [0,0,0]
			startTime=time.time()
			while (time.time()-startTime) < 1:
				totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
				counter = counter + 1.0
				time.sleep(dt)

			forceBias = vectorops.mul(totalF,1.0/float(counter))
			#print forceBias


			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/force'+str(i)+'.txt','w')
			#numberData=open('numberData'+str(i)+'.txt','w')
			### now start collecting data..
			wrench = robotControlApi.getWrench()
			Force = vectorops.sub(wrench[0:3],forceBias)
			
			travel = -0.01
			forceHistory=[Force]
			displacementHistory=[travel]
			stepVector = vectorops.mul(approachVector,moveStep)
			while math.fabs(Force[2]) < forceLimit:
				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
					if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
						robotControlApi.setConfig(klampt_2_controller(robot.getConfig()))
					else:
						print "IK too far away"
						break
				else:
					print "IK failture"
					break
				time.sleep(dt)
				Force = vectorops.sub(robotControlApi.getWrench()[0:3],forceBias)
				travel = travel + moveStep
				forceHistory.append(Force)
				displacementHistory.append(travel)
			# TODO: x, y inverse, reference correct force.py

			#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
			#file specifies the number of datapoints at each detected point
			for (f,d) in zip(forceHistory,displacementHistory):
				forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(d)+'\n')

			#numberData.write(str(len(forceHistory))+'\n')

			### move the probe away

			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=[[probeLength,0,0],[1,0,0],[probeLength,0,1]],world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
				if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
					constantVServo(robotControlApi,shortServoTime,klampt_2_controller(robot.getConfig()),dt)
				else:
					print "IK too far away"
					break
			else:
				print "IK failture"
				break
			forceData.close()
			
			#### move back to intermediate config
			constantVServo(robotControlApi,shortServoTime,intermediateConfig,dt)
			#numberData.close()		
			print'----------------------- pt '+str(i)+' completed -------------------------------'
			
		robotControlApi.stop()

	if CONTROLLER ==  "debugging":
		vis.show()
		while vis.shown():
			time.sleep(1.0)
