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
import math
import random
import matplotlib.pyplot as plt
from scipy import signal
from open3d import *
import os

### Constants and Measurements
#tableHeight = 0.85
#tableHeight = 0.859
#tableHeight = 0.855
'''
tableHeight = 0.865
probeLength = 0.1
forceLimit = 5
dt=0.008
moveStep=0.002*dt   #2mm /s
# note: dt and movestep no use?
'''

def display_inlier_outlier(cloud, ind):
	inlier = select_down_sample(cloud, ind)
	outlier = select_down_sample(cloud, ind, invert = True)

	outlier.paint_uniform_color([1,0,0])
	inlier.paint_uniform_color([0.8,0.8,0.8])
	draw_geometries([inlier,outlier])

def run_collection_process_PCD(config):
	"""
	this is the main function.
	"""

	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt = config.dt
	moveStep = 0.002*dt

	if not os.path.exists(config.exp_path+'exp_'+str(config.exp_number)+"/processed/"):
		os.mkdir(config.exp_path+'exp_'+str(config.exp_number)+"/processed/")

	for i in range(0,config.num_pcd):
		II = i
		####### Process Pcd data ###############
		#originalPcd = read_point_cloud("calibration_data/objectScan_phantom.xyzrgb",format='xyzrgb')
		#originalPcd = read_point_cloud(config.exp_path+'exp_'+str(config.exp_number)+"/objectScan_"+str(II)+".xyzrgb",format='xyzrgb')
		#TODO:
		originalPcd = PointCloud()
		originalPcd_array = np.load(config.exp_path+'exp_'+str(config.exp_number)+"/objectScan_"+str(II)+"_new.npy")
		originalPcd.points = Vector3dVector(originalPcd_array[:,:3])
		originalPcd.colors = Vector3dVector(originalPcd_array[:,3:])

		draw_geometries([originalPcd])

		### remove outlier
		originalPcd,ind=statistical_outlier_removal(originalPcd,nb_neighbors=20,std_ratio=0.1)
		#originalPcd,ind=radius_outlier	_removal(originalPcd,nb_points=5,radius=0.05)
		#display_inlier_outlier(originalPcd, ind)

		draw_geometries([originalPcd])
		#show attributes of originalPcd
		#print dir(originalPcd)
		#estimate the normals

		estimate_normals(originalPcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30)) # normals can actually point downwards......
		#draw_geometries([originalPcd])

		### remove table and points that are too "flat"
		pcdSegmented = PointCloud()
		xyz=[]
		rgb=[]
		ptNormals=[]

		for (indPt,ptColor,ptNormal) in zip(originalPcd.points,originalPcd.colors,originalPcd.normals):
			## normalize the normal
			unitNormal=vectorops.unit(ptNormal)
			if (indPt[2] > tableHeight):# and (abs(unitNormal[2])>0.866): #60 degrees
				xyz.append(indPt)
				rgb.append(ptColor)
				if unitNormal[2] > 0:
					ptNormals.append(ptNormal)
				else:
					ptNormals.append(vectorops.mul(ptNormal,-1.0))

		pcdSegmented.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
		pcdSegmented.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
		pcdSegmented.normals = Vector3dVector(np.asarray(ptNormals,dtype=np.float32))

		draw_geometries([pcdSegmented])

		write_point_cloud(config.exp_path+'exp_'+str(config.exp_number)+"/processed/objectScan_"+str(II)+".pcd",pcdSegmented)

		## save as pcd 

		## Downsample and save
		'''
		### Down sample the points for data collection
		pcdSegmentedDown= voxel_down_sample(pcdSegmented, voxel_size = 0.005)
		#NofPoints=countPts(pcdSegmentedDown)
		#print countPts(pcdSegmentedDown)
		draw_geometries([pcdSegmentedDown])


		################### Need to save this point cloud ##################
		#each row should have position color normal
		pcdData=open('calibration_data/probePcd.txt','w')
		for (indPt,ptColor,ptNormal) in zip(pcdSegmentedDown.points,pcdSegmentedDown.colors,pcdSegmentedDown.normals):
			pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
			pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
			pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+'\n')
		pcdData.close()
		'''
