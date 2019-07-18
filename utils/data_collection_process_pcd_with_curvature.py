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

### Constants and Measurements
#tableHeight = 0.85
'''
tableHeight = 0.86
#tableHeight = 0.92
probeLength = 0.1
forceLimit = 3
dt=0.008
moveStep=0.002*dt   #2mm /s

drawFlag = False
'''

def display_inlier_outlier(cloud, ind):
	inlier = select_down_sample(cloud, ind)
	outlier = select_down_sample(cloud, ind, invert = True)

	outlier.paint_uniform_color([1,0,0])
	inlier.paint_uniform_color([0.8,0.8,0.8])
	draw_geometries([inlier,outlier])

def check_existence(existingList,point,epsilon):
	for element in existingList:
		if vectorops.norm(vectorops.sub(element,point)) < epsilon:
			return True #exists already
	return False

def run_calculation(config):
	tableHeight = config.tableHeight
	probeLength = config.probeLength
	forceLimit = config.forceLimit
	dt = config.dt
	moveStep=0.002*dt   #2mm /s
	drawFlag = config.drawFlag
	path_ori_pcd = config.exp_path+'exp_'+str(config.exp_number)+"/TSDF_converted.ply"
	####### Process Pcd data ###############
	originalPcd = read_point_cloud(path_ori_pcd,format='ply')

	if drawFlag:
		draw_geometries([originalPcd])

	# TODO:the PCD seems to have lots of duplicated points (this is probably a bug in TSDF library..)
	#	
	#
	#
	#####################

	### ------------------ remove outlier
	print len(originalPcd.points)," of points before outlier removal"
	originalPcd,ind=statistical_outlier_removal(originalPcd,nb_neighbors=30,std_ratio=0.5)
	print len(originalPcd.points)," of points after outlier removal"
	NofPtsOriginal = len(originalPcd.points)

	### remove duplicated points
	pcdNoDuplication= voxel_down_sample(originalPcd, voxel_size = 0.0008)
	print NofPtsOriginal-len(pcdNoDuplication.points)," of pts removed due to duplication"


	#### Do this in the future...
	#originalPcd = pcdNoDuplication
	#### For steak_06102019 remove the Duplication on my own..

	pcdSegmented = PointCloud()
	xyz=[]
	rgb=[]
	ptNormals=[]

	print len(originalPcd.points)

	#originalPcd,ind=radius_outlier_removal(originalPcd,nb_points=5,radius=0.05)
	#display_inlier_outlier(originalPcd, ind)
	#draw_geometries([originalPcd])

	### --------------------- estimate normals (using 30 NN)
	estimate_normals(originalPcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30)) # normals can actually point downwards......
	if drawFlag:
		draw_geometries([originalPcd])
	### ---------------------- remove table and points that are too "flat"
	pcdSegmented = PointCloud()
	xyz=[]
	rgb=[]
	ptNormals=[]

	for (indPt,ptColor,ptNormal) in zip(originalPcd.points,originalPcd.colors,originalPcd.normals):
		## normalize the normal
		unitNormal=vectorops.unit(ptNormal)
		if (indPt[2] > tableHeight) and (abs(unitNormal[2])>0.9):
		#if (abs(unitNormal[2])>0.866):
		#if True:
			xyz.append(indPt)
			rgb.append(ptColor)
			if unitNormal[2] > 0:
				ptNormals.append(ptNormal)
			else:
				ptNormals.append(vectorops.mul(ptNormal,-1.0))

	pcdSegmented.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
	pcdSegmented.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
	pcdSegmented.normals = Vector3dVector(np.asarray(ptNormals,dtype=np.float32))

	pcdSegmented,ind=statistical_outlier_removal(pcdSegmented,nb_neighbors=20,std_ratio=2.0)

	print len(pcdSegmented.points)
	draw_geometries([pcdSegmented])

	### ------------------------ Down sample the points for data collection --------
	pcdSegmentedDown= voxel_down_sample(pcdSegmented, voxel_size = 0.01)
	#NofPoints=countPts(pcdSegmentedDown)
	print 'N of Segmented Pts: ',len(pcdSegmentedDown.points)
	draw_geometries([pcdSegmentedDown])

	################### Need to save this point cloud ##################
	#each row should have position color normal
	pcdData=open(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt','w')
	for (indPt,ptColor,ptNormal) in zip(pcdSegmentedDown.points,pcdSegmentedDown.colors,pcdSegmentedDown.normals):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+' ')
		#curvature still needs to be debugges.....
		#[k,idx,_] = pcdTree.search_knn_vector_3d(indPt,NofNN+1)
		#k = curvatures[idx[0]]
		k = 0.0
		pcdData.write(str(k)+'\n')

	##### check repeated points...
	#####
	pcdData.close()

	################### save the original pcd ##################
	#each row should have position color normal
	pcdData=open(config.exp_path+'exp_'+str(config.exp_number)+'/originalPcd.txt','w')
	for (indPt,ptColor,ptNormal) in zip(pcdSegmented.points,pcdSegmented.colors,pcdSegmented.normals):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+'\n')
		#[k,idx,_] = pcdTree.search_knn_vector_3d(indPt,NofNN+1)
		#k = curvatures[idx[0]]
		#pcdData.write(str(k)+'\n')
	pcdData.close()
	print('[*]Processing PCD Done.')

