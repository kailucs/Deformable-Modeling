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

def run_poking(config):
	if config.probe_type == 'point':
		run_poking_point_probe(config)
	elif config.probe_type == 'line':
		run_poking_line_probe(config)
	else:
		print('[!]Probe type no exist')

def run_poking_point_probe(config):
	"""
	this is the main function of poking object. - point probe
	"""
	##################################### Start here ################################
	## Constants and Measurements
	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt=config.dt  #250Hz
	moveStep=0.002*dt   #2mm /s

	shortServoTime=config.shortServoTime
	longServoTime=config.longServoTime + 2 #TODO: set in config?
	IKErrorTolerence=config.IKErrorTolerence
	maxDev=config.maxDev
	EEZLimit=config.EEZLimit

	probe_transform = config.probe_transform
	point_probe = np.array([[0,0,0,1],
							[1-probeLength,0,0,1],
							[0,0,1,1]]) # means the point in probe coordinate.
	point_probe_to_local = np.dot(probe_transform, point_probe.T)
	point_probe_to_local = point_probe_to_local[0:3,:].T
	point_probe_to_local = point_probe_to_local.tolist()
	print("[*]Debug: probe coodinate transform to EE:")
	print(point_probe_to_local)

	## Initialize things 
	world = WorldModel()
	res = world.readFile(config.robot_model_path)
	robot = world.robot(0)
	ee_link=config.ee_link_number #UR5 model is 7.
	link=robot.link(ee_link)
	CONTROLLER = config.mode

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

	# control interface
	if CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
	print '---------------------robot started -----------------------------'

	## Record some home configuration
	intermediateConfig = config.intermediateConfig
	homeConfig2 = intermediateConfig

	if CONTROLLER == "physical":
		constantVServo(robotControlApi,4,homeConfig2,dt)#controller format

	robot.setConfig(controller_2_klampt(robot,homeConfig2))
	print '---------------------at home configuration -----------------------------'

	if CONTROLLER == 'debugging':
		vis.add("world",world)
		differences=[]
		print('[*]Debug: Poking process start')
		#for i in range(len(points)):
		#this is a little stupid...
		point_list = input('There are %d poking point, input a list:'%len(points))
		if point_list == len(points):
			point_list = range(len(points))
		
		for i in point_list:
			print('point %d'%i)
			print points[i],normals[i]
			robotCurrentConfig=homeConfig2
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0)) #get unit vector in the direction '- normals'

			## perform IK
			localZUnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))
			
			pt1=goalPosition
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.			
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)

			# collide detect
			collider = collide.WorldCollider(world)
			if check_collision_linear(robot,collider,controller_2_klampt(robot,robotCurrentConfig),robot.getConfig(),10):
				print "[!]collision!"
			else:
				print "No Collision."
				
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
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
			travel = 0.0
			stepVector = vectorops.mul(approachVector,moveStep)
			
			while travel<0.0001: #just try 0.1mm?
				robotCurrentConfig=klampt_2_controller(robot.getConfig())
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
					if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
						pass
					else:
						print "IK too far away"
						break
				else:
					print "IK failture"
					break
				travel = travel + moveStep

			### move the probe away
			robotCurrentConfig=klampt_2_controller(robot.getConfig())
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
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
		
		vis.show()
		while vis.shown():
			time.sleep(1.0)

	elif CONTROLLER == 'physical':
		######################################## Ready to Take Measurements ################################################
		#TODO: arrange the point execute order
		exe_number = input('There are %d poking point, input executing number:'%len(points))
		point_list = random.sample(range(len(points)),exe_number)

		for i in point_list:	
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			#calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))
			#### Make sure no contact, backup 0.01m
			localZUnitV=vectorops.unit(vectorops.cross([0,1,0],approachVector))

			pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,-0.01))
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))			
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
			res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
			if res:
				diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
				EEZPos=link.getTransform()[1]
				if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
					constantVServo(robotControlApi,longServoTime,klampt_2_controller(robot.getConfig()),dt)
				else:
					print "IK too far away"
					break
			else:
				print "IK failture"
				break

			time.sleep(0.2)

			## Zero the sensor before straight line push
			#
			#
			# Note that the force is recorded in the global frame..
			# And the global frame has x and y axis flipped w.r.t the URDF....

			counter = 0.0
			totalF = [0,0,0]
			startTime=time.time()
			while (time.time()-startTime) < 1: # use 1s to cal the Force
				totalF = vectorops.add(totalF,robotControlApi.getWrench()[0:3])
				counter = counter + 1.0
				time.sleep(dt)
			forceBias = vectorops.mul(totalF,1.0/float(counter)) # when probe no touch the obj, F_avr = sum(F)/n

			### now start collecting data..
			# Force direction x, y inverse, refer to correct force.py
			wrench = robotControlApi.getWrench()
			Force = vectorops.sub(wrench[0:3],forceBias)
			Force = fix_direction(Force)
			Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
			travel = -0.01
			
			forceHistory = [Force]			
			force_normalHistory = [Force_normal]
			displacementHistory = [travel]
			stepVector = vectorops.mul(approachVector,moveStep)

			while Force_normal < forceLimit:
				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(pt1,stepVector)
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
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
				Force = fix_direction(Force)
				Force_normal = math.fabs(vectorops.dot(Force,approachVector))
				travel = travel + moveStep

				forceHistory.append([Force[0],Force[1],Force[2]])				
				force_normalHistory.append(Force_normal)
				displacementHistory.append(travel)
			
			#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
			#file specifies the number of datapoints at each detected point
			forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/force_'+str(i)+'.txt','w')
			for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
				forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')

			### move the probe away

			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
			pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
			pt3=vectorops.add(pt1,localZUnitV)

			goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
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
			print'----------------------- pt '+str(i)+' completed -------------------------------'
			
		robotControlApi.stop()

def run_poking_line_probe(config):
	"""
	this is the main function of poking object. line probe
	"""
	##################################### Start here ################################
	## Constants and Measurements
	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt=config.dt  #250Hz
	moveStep=0.002*dt   #2mm /s

	shortServoTime=config.shortServoTime
	longServoTime=config.longServoTime + 2 #TODO: set in config?
	IKErrorTolerence=config.IKErrorTolerence
	maxDev=config.maxDev
	EEZLimit=config.EEZLimit

	probe_transform = config.probe_transform
	point_probe = np.array([[0,0,0,1],
							[1-probeLength,0,0,1],
							[0,0,1,1]]) # means the point in probe coordinate.
	point_probe_to_local = np.dot(probe_transform, point_probe.T)
	point_probe_to_local = point_probe_to_local[0:3,:].T
	point_probe_to_local = point_probe_to_local.tolist()
	print("[*]Debug: probe coodinate transform to EE:")
	print(point_probe_to_local)

	## Initialize things 
	world = WorldModel()
	res = world.readFile(config.robot_model_path)
	robot = world.robot(0)
	ee_link=config.ee_link_number #UR5 model is 7.
	link=robot.link(ee_link)
	CONTROLLER = config.mode

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

	# control interface
	if CONTROLLER == 'physical':
		robotControlApi = UR5WithGripperController(host=config.robot_host,gripper=False)
		robotControlApi.start()
		time.sleep(2)
	print '---------------------robot started -----------------------------'

	## Record some home configuration
	intermediateConfig = config.intermediateConfig
	homeConfig2 = intermediateConfig

	if CONTROLLER == "physical":
		constantVServo(robotControlApi,4,homeConfig2,dt)#controller format

	robot.setConfig(controller_2_klampt(robot,homeConfig2))
	print '---------------------at home configuration -----------------------------'

	if CONTROLLER == 'debugging':
		vis.add("world",world)
		differences=[]
		print('[*]Debug: Poking process start')
		#for i in range(len(points)):
		# this is a little stupid...
		point_list = input('There are %d poking point, input a list:'%len(points))
		if point_list == len(points):
			point_list = range(len(points))
		
		for i in point_list:
			print('point %d'%i)
			print points[i],normals[i]
			robotCurrentConfig=homeConfig2
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0)) #get unit vector in the direction '- normals'

			## random a theta list for each point
			theta_list_num = input('theta list number: ')
			theta_list = [((math.pi/2)*i/theta_list_num - math.pi/4) for i in range(theta_list_num)]
			#print(theta_list)
			
			for theta in theta_list:
				## perform IK
				localZUnitV=vectorops.unit(vectorops.cross([-math.sin(theta),math.cos(theta),0],approachVector)) # suppose is the right hand coordinate

				pt1=goalPosition
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.				
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)

				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
					print('[*]IK: difference: %f, with theta: %f'%(diff,theta))	
					differences.append(diff)

					vis.add("ghost"+str(i),robot.getConfig())
					vis.setColor("ghost"+str(i),0,1,0,0.5)	

					if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
						pass
					else:
						print "IK too far away"
						#break
				else:
					print "IK failture"
					#break

				### now start colecting data..
				travel = 0.0
				stepVector = vectorops.mul(approachVector,moveStep)
				
				while travel<0.0001: #just try 0.1mm?
					robotCurrentConfig=klampt_2_controller(robot.getConfig())
					pt1=vectorops.add(pt1,stepVector)
					pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
					pt3=vectorops.add(pt1,localZUnitV)

					goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
					res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
					if res:
						diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
						EEZPos=link.getTransform()[1]
						if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
							pass
						else:
							print "IK too far away"
							#break
					else:
						print "IK failture"
						#break
					travel = travel + moveStep

				### move the probe away
				robotCurrentConfig=klampt_2_controller(robot.getConfig())
				pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)
				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
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
		
		vis.show()
		while vis.shown():
			time.sleep(1.0)

	elif CONTROLLER == 'physical':
		######################################## Ready to Take Measurements ################################################
		#TODO: arrange the point execute order
		exe_number = input('There are %d poking point, input executing number:'%len(points))
		point_list = random.sample(range(len(points)),exe_number)
		theta_bias = config.probe_line_theta_bias

		for i in point_list:	
			robotCurrentConfig=robotControlApi.getConfig()
			robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
			## calculate start position
			goalPosition=deepcopy(points[i])
			approachVector=vectorops.unit(vectorops.mul(normals[i],-1.0))

			## random a theta list for each point, theta means the angle of local z axis -> global x-y surface. 
			theta_list_num = input('theta list number: ')
			theta_list = [math.pi*i/theta_list_num for i in range(theta_list_num)]

			for theta in theta_list:

				localZUnitV=vectorops.unit(vectorops.cross([-math.sin(theta),math.cos(theta),0],approachVector)) # suppose is the right hand coordinate

				#### Make sure no contact, backup 0.01m
				pt1=vectorops.add(goalPosition,vectorops.mul(approachVector,-0.01))
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength)) # use 1m in normals direction.				
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
				res=ik.solve_nearby(goal,maxDeviation=maxDev,tol=0.00001)

				if res:
					diff=vectorops.norm_L1(vectorops.sub(robotCurrentConfig,klampt_2_controller(robot.getConfig())))
					EEZPos=link.getTransform()[1]
					if diff<IKErrorTolerence and EEZPos>EEZLimit:  #126 degrees
						constantVServo(robotControlApi,longServoTime,klampt_2_controller(robot.getConfig()),dt)
					else:
						print "IK too far away"
						break
				else:
					print "IK failture"
					break

				time.sleep(0.2)

				## Zero the sensor before straight line push
				#
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
				Force = vectorops.sub(wrench[0:3],forceBias)
				Force = fix_direction(Force)
				Torque = vectorops.sub(wrench[3:6],torqueBias)
				Torque = fix_direction(Torque)
				Force_normal = math.fabs(vectorops.dot(Force,approachVector)) #|F||n|cos(theta) = F dot n, set it >= 0 
				
				#theta_probe = theta-theta_bias
				#localZUnitV_fix=vectorops.cross([-math.sin(theta_probe),math.cos(theta_probe),0],approachVector)
				## TODO: no need, if debug ok, then delete!
				Torque_normal = vectorops.dot(Torque,localZUnitV)

				travel = -0.01
				
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
					pt3=vectorops.add(pt1,localZUnitV)

					goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
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
					Force = fix_direction(Force)
					Torque = vectorops.sub(robotControlApi.getWrench()[3:6],torqueBias)
					Torque = fix_direction(Torque)
					Force_normal = math.fabs(vectorops.dot(Force,approachVector))
					Torque_normal = vectorops.dot(Torque,localZUnitV)

					travel = travel + moveStep

					forceHistory.append([Force[0],Force[1],Force[2]])				
					force_normalHistory.append(Force_normal)
					torqueHistory.append([Torque[0],Torque[1],Torque[2]])				
					torque_normalHistory.append(Torque_normal)

					displacementHistory.append(travel)
				
				#record all the data in 2 files, one N*2 containts all the force data collected at various locations, another
				#file specifies the number of datapoints at each detected point
				forceData=open(config.exp_path+'exp_'+str(config.exp_number)+'/force_'+str(i)+'.txt','w')
				for (f,fn,d) in zip(forceHistory,force_normalHistory,displacementHistory):
					forceData.write(str(f[0])+' '+str(f[1])+' '+str(f[2])+' '+str(fn)+' '+str(d)+'\n')
				
				torqueData=open(config.exp_path+'exp_'+str(config.exp_number)+'/torque_'+str(i)+'.txt','w')
				for (t,tn,d) in zip(torqueHistory,torque_normalHistory,displacementHistory):
					torqueData.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+' '+str(tn)+' '+str(d)+'\n')

				### move the probe away

				robotCurrentConfig=robotControlApi.getConfig()
				robot.setConfig(controller_2_klampt(robot,robotCurrentConfig))
				pt1=vectorops.add(points[i],vectorops.mul(approachVector,-0.05))  ## move the probe 5 cm from the object surface
				pt2=vectorops.add(pt1,vectorops.mul(approachVector,1.0-probeLength))
				pt3=vectorops.add(pt1,localZUnitV)

				goal=ik.objective(link,local=point_probe_to_local,world=[pt1,pt2,pt3])
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
				torqueData.close()

				#### move back to intermediate config
				constantVServo(robotControlApi,shortServoTime,intermediateConfig,dt)	
				print'----------------------- pt '+str(i)+' completed -------------------------------'
			
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
	Force[0] = -Force[0]
	Force[1] = -Force[1]
	return Force

def drop_code():
	"""
	drop code
	"""
	
	"""

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
	#import logging
	#logging.basicConfig(level=logging.INFO)

	if CONTROLLER == 'simulation':
		controllerWorld = world.copy()
	
	"""


