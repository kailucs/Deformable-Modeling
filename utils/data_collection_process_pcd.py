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
		originalPcd = PointCloud()
		originalPcd_array = np.load(config.exp_path+'exp_'+str(config.exp_number)+"/objectScan_"+str(II)+"_new.npy")
		
		# rm table
		originalPcd_array = originalPcd_array[originalPcd_array[:,2]>tableHeight]
		# rm black bad points
		originalPcd_array = originalPcd_array[np.sum(originalPcd_array[:,3:6],axis=1)!=0]
		#TODO: very important!

		originalPcd.points = Vector3dVector(originalPcd_array[:,:3])
		originalPcd.colors = Vector3dVector(originalPcd_array[:,3:])
		#draw_geometries([originalPcd])

		#print(len(originalPcd.points))
		### remove outlier
		originalPcd,ind=statistical_outlier_removal(originalPcd,nb_neighbors=10,std_ratio=0.1)
		draw_geometries([originalPcd])
		print(len(originalPcd.points))
		
		estimate_normals(originalPcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30)) # normals can actually point downwards......

		### remove table and points that are too "flat"
		pcdSegmented = PointCloud()

		#speed up with numpy
		st = time.time()
		xyz = np.array(originalPcd.points)
		rgb = np.array(originalPcd.colors)
		
		n = np.array(originalPcd.normals)
		n_s = np.sqrt(np.sum(pow(n,2),axis=1))
		n_s = np.reshape(n_s,(n_s.shape[0],1))
		n_s = np.hstack((n_s,n_s,n_s))
		n = n/n_s #normalize #TODO: NEED?
		n[n[:,2]<0] = - n[n[:,2]<0] #abs nz #TODO:
		
		xyzrgbn = np.hstack((xyz,rgb,n))
		#xyzrgbn = xyzrgbn[xyzrgbn[:,2]>tableHeight] #rm table

		pcdSegmented.points = Vector3dVector(xyzrgbn[:,:3])
		pcdSegmented.colors = Vector3dVector(xyzrgbn[:,3:6])
		pcdSegmented.normals = Vector3dVector(xyzrgbn[:,6:9])
		et = time.time()
		print('[*]rm desk and fix normal direction in %fs'%(et-st))

		draw_geometries([pcdSegmented])

		write_point_cloud(config.exp_path+'exp_'+str(config.exp_number)+"/processed/objectScan_"+str(II)+".pcd",pcdSegmented)
		
		#TODO:
		'''
		if II!= 0:
			center = [-0.50,0.105]
			xyzrgbn = xyzrgbn[((xyzrgbn[:,0]-center[0])**2+(xyzrgbn[:,1]-center[1])**2)**0.5<0.03]

			pcdSegmented.points = Vector3dVector(xyzrgbn[:,:3])
			pcdSegmented.colors = Vector3dVector(xyzrgbn[:,3:6])
			pcdSegmented.normals = Vector3dVector(xyzrgbn[:,6:9])
			draw_geometries([pcdSegmented])

			print(i)

			write_point_cloud(config.exp_path+'exp_'+str(config.exp_number)+"/processed/objectScan_"+str(II)+".pcd",pcdSegmented)
		## save as pcd 
		'''