import time
import numpy as np
import cv2
from copy import deepcopy
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
from klampt import vis
from klampt.model import collide
import math
import random
from robot_api.RobotController import UR5WithGripperController
import matplotlib.pyplot as plt
from scipy import signal
from utils.collision_detecting import check_collision_single,check_collision_linear
import os

###
def run_poking(config):
	"""
	this is poking api entrance.
	"""
	# init params
	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt=config.dt  #250Hz
	moveStep=0.002*dt   #2mm /s

	shortServoTime=config.shortServoTime
	longServoTime=config.longServoTime
	IKErrorTolerence=config.IKErrorTolerence
	maxDev=config.maxDev
	EEZLimit=config.EEZLimit

	intermediateConfig = config.intermediateConfig

	probe_transform = config.probe_transform
	point_probe = np.array([[0,0,0,1],
							[1-probeLength,0,0,1],
							[0,-1,0,1]]) # means the point in probe coordinate.

	point_probe_to_local = np.dot(probe_transform, point_probe.T)
	point_probe_to_local = point_probe_to_local[0:3,:].T
	point_probe_to_local = point_probe_to_local.tolist()
	print("[*]Debug: probe coodinate transform to EE:")
	print(point_probe_to_local)


	# init robot 
	world = WorldModel()
	res = world.readFile(config.robot_model_path)
	robot = world.robot(0)
	ee_link=config.ee_link_number #UR5 model is 7.
	link=robot.link(ee_link)
	CONTROLLER = config.mode
	collider = collide.WorldCollider(world)
	print '---------------------model loaded -----------------------------' 

	# visualization
	vis.add("world",world)

	# create folder
	data_folder = config.exp_path+'exp_'+str(config.exp_number)+'/'+config.probe_type
	if not os.path.exists(data_folder):
		os.mkdir(data_folder)

	# begin loop
	if config.probe_type == 'point':
		run_poking_point_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
								IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,robot,link,CONTROLLER,collider,intermediateConfig)
	elif config.probe_type == 'line':
		run_poking_line_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
								IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,robot,link,CONTROLLER,collider,intermediateConfig)
	elif config.probe_type == 'ellipse':
		run_poking_ellipse_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
								IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,robot,link,CONTROLLER,collider,intermediateConfig)
	else:
		print('[!]Probe type no exist')

def run_poking_point_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
								IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,
								robot,link,CONTROLLER,collider,intermediateConfig):
	"""
	this is the main function of poking object. - point probe
	"""
	# Read In the pcd 
	points, normals = load_pcd(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt')
	
	# control interface
	if CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
		print '---------------------robot started -----------------------------'
		constantVServo(robotControlApi,4,intermediateConfig,dt)#controller format

	# in simulation ,set
	robot.setConfig(controller_2_klampt(robot,intermediateConfig))
	print '---------------------at home configuration -----------------------------'

	if CONTROLLER == 'debugging':
		differences=[]
		print('[*]Debug: Poking process start!')

		for i in range(len(points)):
			print('point %d, pos: %s, normals: %s'%(i,points[i],normals[i]))
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0)) #get unit vector in the direction '- normals'

			## perform IK
			local_NY_UnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))			
			pt1=goalPosition
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.			
			pt3=vectorops.add(pt1,local_NY_UnitV)

			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
											IKErrorTolerence,EEZLimit,collider,use_collision_detect=True)
			differences.append(difference)
			print('difference: %f'%difference)

			### now start colecting data..
			travel = 0.0
			stepVector = vectorops.mul(approachVector,moveStep)
			
			while travel<0.0001: #just try 0.1mm?
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
												IKErrorTolerence,EEZLimit,collider,use_const=False)
				travel = travel + moveStep
				
			### move the probe away, note: a bit different to physical mode
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,local_NY_UnitV)
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
											IKErrorTolerence,EEZLimit,collider)

			### move back to intermediate config
			robot.setConfig(controller_2_klampt(robot,intermediateConfig))
		
		print('[*]Debug: Poking process done, with max difference:%f'%max(differences))
		
		vis.show()
		while vis.shown():
			time.sleep(1.0)

	elif CONTROLLER == 'physical':
		input('There are %d poking point, go?'%len(points))
		point_list = range(112,116) # !delete 85, 90
		#point_list = random.sample(range(97),97)

		#point_list = [40,37,67,68]
		for i in point_list:	
			print('point %d, pos: %s, normals: %s'%(i,points[i],normals[i]))
			
			travel = -0.01
			#init record file
			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/point/force_'+str(i)+'.txt','w')

			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			#calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))
			#### Make sure no contact, backup 0.01m
			local_NY_UnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))

			pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,travel))
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))			
			pt3=vectorops.add(pt1,local_NY_UnitV)

			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime-1,dt) #TODO:

			time.sleep(0.2)

			# Zero the sensor before straight line push, Note that the force is recorded in the global frame..
			counter = 0.0
			totalF = [0,0,0]
			startTime=time.time()
			while (time.time()-startTime) < 1: # use 1s to cal the Force
				totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
				counter = counter + 1.0
				time.sleep(dt)
			forceBias = vectorops.mul(totalF,1.0/float(counter)) # when probe no touch the obj, F_avr = sum(F)/n

			### now start collecting data..
			wrench = robotControlApi.getWrench()
			Force = vectorops.sub(wrench[0:3],forceBias)
			Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
						
			forceHistory = [Force]			
			force_normalHistory = [Force_normal]
			displacementHistory = [travel]
			stepVector = vectorops.mul(approachVector,moveStep)

			while Force_normal < forceLimit:
				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
												maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime,dt,use_const=False)
				time.sleep(dt)

				Force = vectorops.sub(robotControlApi.getWrench()[0:3],forceBias)
				Force_normal = math.fabs(vectorops.dot(Force,approachVector))
				travel = travel + moveStep

				forceHistory.append([Force[0],Force[1],Force[2]])				
				force_normalHistory.append(Force_normal)
				displacementHistory.append(travel)
			
			#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
			#file specifies the number of datapoints at each detected point
			for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
				forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')
			forceData.close()

			### move the probe away
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.10))  ## move the probe 8 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,local_NY_UnitV)
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,shortServoTime,dt)
			
			#constantVServo(robotControlApi,longServoTime,intermediateConfig,dt)

			print'----------------------- pt '+str(i)+' completed -------------------------------'
		
		#### move back to intermediate config
		constantVServo(robotControlApi,shortServoTime,intermediateConfig,dt)	
		robotControlApi.stop()

def run_poking_line_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
							IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,
							robot,link,CONTROLLER,collider,intermediateConfig):
	"""
	this is the main function of poking object. - line probe
	"""
	# reconstruct probepcd.txt
	if input('[*]Reconstruct probe pcd?') == 1:
		theta_list_num = input('---need theta list number: ')
		reconstruct_pcd(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt',
						config.exp_path+'exp_'+str(config.exp_number)+'/probePcd_theta.txt',
						theta_list_num)
		print('---New probe list done')

	# Read In the pcd
	points, normals, theta_list, theta, pti = load_pcd(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd_theta.txt', pcdtype='xyzrgbntheta')
	
	# control interface
	if CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
		print '---------------------robot started -----------------------------'
		constantVServo(robotControlApi,4,intermediateConfig,dt)#controller format

	# set in simul model
	robot.setConfig(controller_2_klampt(robot,intermediateConfig))
	print '---------------------at home configuration -----------------------------'

	if CONTROLLER == 'debugging':
		differences=[]
		print('[*]Debug: Poking process start')

		i = 0 # use this to catch points
		pti_ = pti[i]
		while(i < len(points)):
			robotCurrentConfig=intermediateConfig
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0)) #get unit vector in the direction '- normals'
			_pti = pti_

			if pti[i] == _pti:
				print('point %d, pos: %s, normals: %s, theta: %s, -> %f'%(i,points[i],normals[i],theta_list[i],theta[i]))
				## perform IK				
				local_NY_UnitV = vectorops.unit(back_2_line(approachVector,theta_list[i])) # the probe's line direction

				pt1=goalPosition
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.				
				pt3=vectorops.add(pt1,local_NY_UnitV)

				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
												maxDev,IKErrorTolerence,EEZLimit,collider,use_collision_detect=False,use_ik_detect=True)
				differences.append(difference)
				print('difference: %f'%difference)

				### now start colecting data..
				travel = 0.0
				stepVector = vectorops.mul(approachVector,moveStep)
				
				while travel<0.0001:
					robotCurrentConfig=klampt_2_controller(robot.getConfig())
					pt1=vectorops.add(pt1,stepVector)
					pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
					pt3=vectorops.add(pt1,local_NY_UnitV)

					[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
													maxDev,IKErrorTolerence,EEZLimit,collider,use_collision_detect=False)
					travel = travel + moveStep

				### move the probe away, note: a bit different to physical mode
				robotCurrentConfig=klampt_2_controller(robot.getConfig())
				pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)

				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
												maxDev,IKErrorTolerence,EEZLimit,collider,use_collision_detect=False)

				### move back to intermediate config
				robot.setConfig(controller_2_klampt(robot,intermediateConfig))

				i = i + 1 # important
			else:
				pti_ = pti[i]
		
		print('[*]Debug: Poking process done, with max difference:%f'%max(differences))
		
		vis.show()
		while vis.shown():
			time.sleep(1.0)

	elif CONTROLLER == 'physical':
		exe_number = input('There are %d poking point, go?'%len(points))

		start_i = 0 #72,93,94,97,99,100,101,108,116,125,128,147,148,150,151,152,189,194~197,207~210   !40,37,67,68 -> 111 112 113 120 121 122 201 206
 		end_i = 1 #len(points)
		i = start_i # use this to catch points, set manully! # TODO:
		pti_ = pti[i]
		
		probe_list = random.sample(range(282),282) #18,15
		finish_list= range(16)+range(17,21)+range(25,44)+range(45,51)+range(52,57)+[58,59,60,63,67,68,71,73,75,76,78]+range(81,96)\
					+[97,99,101,103,104,108,110,112,114,115,117,118,120,124,125,128,129,130]+range(132,138)+[141,142,144,147,149,150,151]+range(153,156)\
					+[159,160,161,167,168,170,172,175,176,177,178]+range(180,186)\
					+[189,192,195,196,199,200,201,203]+range(204,210)+range(211,217)+[219]+range(221,229)+range(230,236)+range(237,241)\
					+range(244,250)+[251]+range(254,261)+[262]+range(264,276)+range(277,282)
		probe_list = [x for x in probe_list if x not in finish_list] +[227]

		probe_list = [95]
		
		for i in probe_list:
			print('point %d, pos: %s, normals: %s, theta: %s, -> %f'%(i,points[i],normals[i],theta_list[i],theta[i]))
			
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			# calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))

			# init record file
			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/line/force_'+str(i)+'.txt','w')
			torqueData=open(config.exp_path+'exp_'+str(config.exp_number)+'/line/torque_'+str(i)+'.txt','w')

			travel = -0.025

			## perform IK
			local_NY_UnitV = vectorops.unit(back_2_line(approachVector,theta_list[i])) # the probe's line direction
			#### Make sure no contact, backup 0.01m
			pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,travel))
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.				
			pt3=vectorops.add(pt1,local_NY_UnitV)
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
										maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,shortServoTime,dt) #TODO:
			time.sleep(0.2)

			# Zero the sensor before straight line push, Note that the force is recorded in the global frame..
			counter = 0.0
			totalF = [0,0,0]
			totalTorque = [0,0,0]
			startTime=time.time()
			while (time.time()-startTime) < 1: # use 1s to cal the Force
				totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
				totalTorque = vectorops.add(totalTorque,robotControlApi.getWrench()[3:6])
				counter = counter + 1.0
				time.sleep(dt)
			forceBias = vectorops.mul(totalF,1.0/float(counter)) # when probe no touch the obj, F_avr = sum(F)/n
			torqueBias = vectorops.mul(totalTorque,1.0/float(counter))

			### now start collecting data..
			wrench = robotControlApi.getWrench()
			Force = vectorops.sub(wrench[0:3],forceBias)
			Torque = vectorops.sub(wrench[3:6],torqueBias)
			Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
			
			local_Z_UnitV = vectorops.cross(normals[i],local_NY_UnitV)
			Torque_normal = vectorops.dot(Torque,local_Z_UnitV) #TODO:
			
			forceHistory = [Force]			
			force_normalHistory = [Force_normal]
			torqueHistory = [Torque]
			torque_normalHistory = [Torque_normal]

			displacementHistory = [travel]
			stepVector = vectorops.mul(approachVector,moveStep)

			while Force_normal < forceLimit:
				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)

				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
										maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime,dt,use_const=False)
				time.sleep(dt)

				Force = vectorops.sub(robotControlApi.getWrench()[0:3],forceBias)
				Torque = vectorops.sub(robotControlApi.getWrench()[3:6],torqueBias)
				Force_normal = math.fabs(vectorops.dot(Force,approachVector))

				local_Z_UnitV = vectorops.cross(normals[i],local_NY_UnitV)
				Torque_normal = vectorops.dot(Torque,local_Z_UnitV)

				travel = travel + moveStep

				forceHistory.append([Force[0],Force[1],Force[2]])				
				force_normalHistory.append(Force_normal)
				torqueHistory.append([Torque[0],Torque[1],Torque[2]])				
				torque_normalHistory.append(Torque_normal)

				displacementHistory.append(travel)
			
			#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
			#file specifies the number of datapoints at each detected point				
			for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
				forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')
			
			for (t,tn,d) in zip(torqueHistory,torque_normalHistory,displacementHistory):
				torqueData.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+' '+str(tn)+' '+str(d)+'\n')

			### move the probe away, sometimes z up 5cm is better than normal direction up 5cm...
			pt1=vectorops.add(pt1,[0,0,0.05])  ## move the probe 10 cm up-z-axis, find another point
			pt2=vectorops.add(pt2,[0,0,0.05])
			pt3=vectorops.add(pt3,[0,0,0.05])
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,shortServoTime-1,dt)

			constantVServo(robotControlApi,longServoTime,intermediateConfig,dt)#TODO:
			# close record file for point i
			forceData.close()
			torqueData.close()
			print'----------------------- pt '+str(i)+' completed -------------------------------'
		'''
		while(i < end_i):
					
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			# calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))

			# init record file
			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/line/force_'+str(i)+'.txt','w')
			torqueData=open(config.exp_path+'exp_'+str(config.exp_number)+'/line/torque_'+str(i)+'.txt','w')

			_pti = pti_
			if pti[i] == _pti:
				print('point %d, pos: %s, normals: %s, theta: %s, -> %f'%(i,points[i],normals[i],theta_list[i],theta[i]))
				
				travel = -0.015

				## perform IK
				local_NY_UnitV = vectorops.unit(back_2_line(approachVector,theta_list[i])) # the probe's line direction
				#### Make sure no contact, backup 0.01m
				pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,travel))
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.				
				pt3=vectorops.add(pt1,local_NY_UnitV)
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime-1,dt) #TODO:
				time.sleep(0.2)

				# Zero the sensor before straight line push, Note that the force is recorded in the global frame..
				counter = 0.0
				totalF = [0,0,0]
				totalTorque = [0,0,0]
				startTime=time.time()
				while (time.time()-startTime) < 1: # use 1s to cal the Force
					totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
					totalTorque = vectorops.add(totalTorque,robotControlApi.getWrench()[3:6])
					counter = counter + 1.0
					time.sleep(dt)
				forceBias = vectorops.mul(totalF,1.0/float(counter)) # when probe no touch the obj, F_avr = sum(F)/n
				torqueBias = vectorops.mul(totalTorque,1.0/float(counter))

				### now start collecting data..
				wrench = robotControlApi.getWrench()
				Force = vectorops.sub(wrench[0:3],forceBias)
				Torque = vectorops.sub(wrench[3:6],torqueBias)
				Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
				
				local_Z_UnitV = vectorops.cross(normals[i],local_NY_UnitV)
				Torque_normal = vectorops.dot(Torque,local_Z_UnitV) #TODO:
				
				forceHistory = [Force]			
				force_normalHistory = [Force_normal]
				torqueHistory = [Torque]
				torque_normalHistory = [Torque_normal]

				displacementHistory = [travel]
				stepVector = vectorops.mul(approachVector,moveStep)

				while Force_normal < forceLimit:
					robotCurrentConfig=robotControlApi.getConfig()
					robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
					pt1=vectorops.add(pt1,stepVector)
					pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
					pt3=vectorops.add(pt1,local_NY_UnitV)

					[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime,dt,use_const=False)
					time.sleep(dt)

					Force = vectorops.sub(robotControlApi.getWrench()[0:3],forceBias)
					Torque = vectorops.sub(robotControlApi.getWrench()[3:6],torqueBias)
					Force_normal = math.fabs(vectorops.dot(Force,approachVector))

					local_Z_UnitV = vectorops.cross(normals[i],local_NY_UnitV)
					Torque_normal = vectorops.dot(Torque,local_Z_UnitV)

					travel = travel + moveStep

					forceHistory.append([Force[0],Force[1],Force[2]])				
					force_normalHistory.append(Force_normal)
					torqueHistory.append([Torque[0],Torque[1],Torque[2]])				
					torque_normalHistory.append(Torque_normal)

					displacementHistory.append(travel)
				
				#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
				#file specifies the number of datapoints at each detected point				
				for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
					forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')
				
				for (t,tn,d) in zip(torqueHistory,torque_normalHistory,displacementHistory):
					torqueData.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+' '+str(tn)+' '+str(d)+'\n')

				### move the probe away, sometimes z up 5cm is better than normal direction up 5cm...
				pt1=vectorops.add(pt1,[0,0,0.05])  ## move the probe 10 cm up-z-axis, find another point
				pt2=vectorops.add(pt2,[0,0,0.05])
				pt3=vectorops.add(pt3,[0,0,0.05])
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
												maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,shortServoTime-0.5,dt)

				#constantVServo(robotControlApi,longServoTime,intermediateConfig,dt)#TODO:
				i = i + 1
				# close record file for point i
				forceData.close()
				torqueData.close()
				print'----------------------- pt '+str(i)+' completed -------------------------------'
			
			else:
				# up 10cm is faster but not good.
				# since the points are close, no need to go back home
			
				constantVServo(robotControlApi,longServoTime,intermediateConfig,dt)
				pti_ = pti[i]
		'''
			
		#### move back to intermediate config
		constantVServo(robotControlApi,shortServoTime,intermediateConfig,dt)
		# finish all points
		robotControlApi.stop()

def run_poking_ellipse_probe(config,tableHeight,probeLength,forceLimit,dt,moveStep,shortServoTime,longServoTime,
								IKErrorTolerence,maxDev,EEZLimit,probe_transform,point_probe_to_local,world,res,
								robot,link,CONTROLLER,collider,intermediateConfig):
	"""
	this is the main function of poking object. - point probe
	"""
	########################## Read In the pcd ######################################
	points, normals = load_pcd(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt')
	
	# control interface
	if CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
	print '---------------------robot started -----------------------------'

	## Record some home configuration
	intermediateConfig = config.intermediateConfig
	intermediateConfig = intermediateConfig

	if CONTROLLER == "physical":
		constantVServo(robotControlApi,4,intermediateConfig,dt)#controller format

	robot.setConfig(controller_2_klampt(robot,intermediateConfig))
	print '---------------------at home configuration -----------------------------'

	if CONTROLLER == 'debugging':
		differences=[]
		print('[*]Debug: Poking process start!')

		for i in range(len(points)):
			print('point %d, pos: %s, normals: %s'%(i,points[i],normals[i]))

			#robotCurrentConfig=intermediateConfig # TODO: compare to the intermediateConfig, I comment it 
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0)) #get unit vector in the direction '- normals'

			## perform IK
			local_NY_UnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))			
			pt1=goalPosition
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.			
			pt3=vectorops.add(pt1,local_NY_UnitV)

			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
											IKErrorTolerence,EEZLimit,collider,use_collision_detect=True,use_ik_detect=True)
			differences.append(difference)
			print('difference: %f'%difference)

			### now start colecting data..
			travel = 0.0
			stepVector = vectorops.mul(approachVector,moveStep)
			
			while travel<0.0001: #just try 0.1mm?
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
												IKErrorTolerence,EEZLimit,collider,use_const=False)
				travel = travel + moveStep
				
			### move the probe away, note: a bit different to physical mode
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,local_NY_UnitV)
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],maxDev,
											IKErrorTolerence,EEZLimit,collider)

			### move back to intermediate config
			robot.setConfig(controller_2_klampt(robot,intermediateConfig))
		
		print('[*]Debug: Poking process done, with max difference:%f'%max(differences))
		
		vis.show()
		while vis.shown():
			time.sleep(1.0)

	elif CONTROLLER == 'physical':
		######################################## Ready to Take Measurements ################################################
		input('[!]Warning: There are %d poking point, Robot act!:'%len(points))
		point_list = range(65,120) #64
		#point_list = random.sample(range(0,94),94)

		#finish_list = [0,1,4,9,10,11,12,13,14,15,17,18,19,20,25,26,28,30,33,34,35,37,42,43,44,46,47,50,51,53,54,57,58,59,64,69,72,73,74,75,76,77,78,79,81,83,86,95]
		#point_list = [x for x in point_list if x not in finish_list]
		
		point_list = [64]
		
		for i in point_list:	
			print('point %d, pos: %s, normals: %s'%(i,points[i],normals[i]))
			
			#init record file
			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/ellipse/force_'+str(i)+'.txt','w')
			torqueData=open(config.exp_path+'exp_'+str(config.exp_number)+'/ellipse/torque_'+str(i)+'.txt','w')

			#init the backforward distance
			travel = -0.018

			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			#calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))
			#### Make sure no contact, backup 0.01m
			local_NY_UnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))

			pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,travel))
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))			
			pt3=vectorops.add(pt1,local_NY_UnitV)

			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime-1,dt,
											use_ik_detect=False,use_collision_detect=False)

			time.sleep(0.2)

			## Zero the sensor before straight line push
			#
			# Note that the force is recorded in the global frame..
			# And the global frame has x and y axis flipped w.r.t the URDF....

			counter = 0.0
			totalF = [0,0,0]
			totalTorque = [0,0,0]

			startTime=time.time()
			while (time.time()-startTime) < 1: # use 1s to cal the Force
				totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
				totalTorque = vectorops.add(totalTorque,robotControlApi.getWrench()[3:6])
				counter = counter + 1.0
				time.sleep(dt)
			forceBias = vectorops.mul(totalF,1.0/float(counter)) # when probe no touch the obj, F_avr = sum(F)/n
			torqueBias = vectorops.mul(totalTorque,1.0/float(counter))


			### now start collecting data..
			# Force direction x, y inverse, refer to correct force.py
			wrench = robotControlApi.getWrench()
			Force = fix_direction(vectorops.sub(wrench[0:3],forceBias))
			Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
			Torque = vectorops.sub(wrench[3:6],torqueBias)
			Torque = fix_direction(Torque)
						
			forceHistory = [Force]
			torqueHistory = [Torque]			
			force_normalHistory = [Force_normal]
			displacementHistory = [travel]
			stepVector = vectorops.mul(approachVector,moveStep)

			while Force_normal < forceLimit:
				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,local_NY_UnitV)
				[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
												maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,longServoTime,dt,
												use_const=False,use_ik_detect=False)
				time.sleep(dt)

				Force = fix_direction(vectorops.sub(robotControlApi.getWrench()[0:3],forceBias))
				Force_normal = math.fabs(vectorops.dot(Force,approachVector))
				Torque = vectorops.sub(robotControlApi.getWrench()[3:6],torqueBias)

				travel = travel + moveStep

				forceHistory.append([Force[0],Force[1],Force[2]])
				torqueHistory.append([Torque[0],Torque[1],Torque[2]])					
				force_normalHistory.append(Force_normal)
				displacementHistory.append(travel)
			
			#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
			#file specifies the number of datapoints at each detected point
			for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
				forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')		
			for (t,d) in zip(torqueHistory,displacementHistory):
				torqueData.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+' '+str(d)+'\n')
			forceData.close()
			torqueData.close()

			### move the probe away
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.10))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,local_NY_UnitV)
			[robot,difference] = robot_move(CONTROLLER,world,robot,link,point_probe_to_local,[pt1,pt2,pt3],
											maxDev,IKErrorTolerence,EEZLimit,collider,robotControlApi,shortServoTime,dt,
											use_ik_detect=False)
			
			#constantVServo(robotControlApi,longServoTime,intermediateConfig,dt)

			print'----------------------- pt '+str(i)+' completed -------------------------------'
		
		#### move back to intermediate config
		constantVServo(robotControlApi,shortServoTime,intermediateConfig,dt)	
		robotControlApi.stop()

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

def fix_direction(Force):
	Force[0] = Force[0]
	Force[1] = Force[1]
	return Force

def robot_move(mode,world,robot,link,point_ee,point_world,maxDev,IKErrorTolerence,
				EEZLimit,collider,robotControlApi=None,ServoTime=9999.0,dt=1.0,
				use_const = True,vis=vis,use_collision_detect = False,use_ik_detect = False):

	robotCurrentConfig=klampt_2_controller(robot.getConfig())
	goal=ik.objective(link,local=point_ee,world=point_world)
	res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
	#res=ik.solve_global(goal,tol=0.00001)

	if res:
		# collision detect
		if check_collision_linear(robot,collider,controller_2_klampt(robot,robotCurrentConfig),robot.getConfig(),10):
			print "[!]Warning: collision detected!"
			if use_collision_detect == True:
				vis.show()
				if input('continue?') != 1:
					exit()
		else:
			pass

		# cal difference
		
		diff=np.max(np.absolute((np.array(vectorops.sub(robotCurrentConfig[0:5],klampt_2_controller(robot.getConfig())[0:5])))))
		EEZPos=link.getTransform()[1]
		if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
			if mode == 'debugging':
				pass
			elif mode == 'physical':
				if use_const:
					constantVServo(robotControlApi,ServoTime,klampt_2_controller(robot.getConfig()),dt)
				else:
					robotControlApi.setConfig(klampt_2_controller(robot.getConfig()))
		else:
			print "[!]IK too far away"
			if use_ik_detect == True:
				if input('continue?') != 1:
					exit()
	else:
		diff = 9999.0
		print "[!]IK failture"
		if use_ik_detect == True:
			vis.show()
			if input('continue?') != 1:
				exit()

	return robot, diff

def load_pcd(path, pcdtype='xyzrgbn'):
	points=[]
	normals=[]
	normal_theta=[]
	theta=[]
	pt_index=[]
	dataFile=open(path,'r')
	for line in dataFile:
		line=line.rstrip()
		l=[num for num in line.split(' ')]
		l2=[float(num) for num in l]
		points.append(l2[0:3])
		normals.append(l2[6:9])
		if pcdtype == 'xyzrgbntheta':
			normal_theta.append(l2[10:13])
			theta.append(l2[13])
			pt_index.append(l2[14])
	dataFile.close()
	print '---------------------pcd loaded -----------------------------'
	if pcdtype == 'xyzrgbn':
		return points, normals
	elif pcdtype == 'xyzrgbntheta':
		return points, normals, normal_theta, theta, pt_index

def reconstruct_pcd(oripath,newpath,theta_list_num):
	oriFile=open(oripath,'r')
	newFile=open(newpath,'w')

	pt_index=0
	for line in oriFile:
		line = line.rstrip()
		l=[num for num in line.split(' ')]
		tmp_list = random.sample(range(100+1),theta_list_num) #TODO:
		theta_list = [(math.pi*tmp/100 - math.pi*(0.0/4.0)) for tmp in tmp_list]
		for theta in theta_list:
			normal_theta = [math.cos(theta),math.sin(theta),0] # means the line probe's line direction
			
			newFile.write(str(l[0])+' '+str(l[1])+' '+str(l[2])+' '+str(l[3])+' '+str(l[4])+' '+
						str(l[5])+' '+str(l[6])+' '+str(l[7])+' '+str(l[8])+' '+str(l[9])+' '+
						str(normal_theta[0])+' '+str(normal_theta[1])+' '+str(normal_theta[2])+' '+
						str(theta)+' '+str(pt_index)+'\n')
		pt_index = pt_index + 1

	oriFile.close()
	newFile.close()

def back_2_line(normal, projection):
	projection[2] = -(normal[0]*projection[0]+normal[1]*projection[1])/normal[2]
	return projection 
