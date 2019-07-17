import logging
logging.basicConfig(level=logging.INFO)
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
tableHeight = 0.86
#tableHeight = 0.92
probeLength = 0.1
forceLimit = 3
dt=0.008
moveStep=0.002*dt   #2mm /s


drawFlag = False


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

def run_calculation()
	####### Process Pcd data ###############




	originalPcd = read_point_cloud("experiment_data_steak_06102019/TSDF_converted.ply",format='ply')


	if drawFlag:
		draw_geometries([originalPcd])

	# the PCD seems to have lots of duplicated points (this is probably a bug in TSDF library..)
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

	'''
	### ---------------------------- Estimate curvature.. Tune parameters here
	#  seems to still have some bug.....
	#
	#
	#
	NofPts=len(pcdSegmented.points)
	NofNN=20
	searchR=0.0025

	pcdTree = KDTreeFlann(pcdSegmented)
	curvatures=[]

	# Below for debugging purposes
	#[k,idx,_] = pcdTree.search_knn_vector_3d([0.05,0,1],1)
	#idx = [0]
	#np.asarray(pcdSegmented.colors)[idx,:] = [0,1,0]
	#print idx

	pcdSegmented.colors[0] =  [0,0,1]
	draw_geometries([pcdSegmented])


	print 'calculating curvatures......'
	#for i in range(NofPts):
	for i in [0]:
		#[k,idx,_] = pcdTree.search_knn_vector_3d(pcdSegmented.points[i],NofNN+1)
		[k,idx,_] = pcdTree.search_radius_vector_3d(pcdSegmented.points[i],searchR)
		#k is the total Number, while idx is the list of indices
		print idx

		for ii in idx:
			pcdSegmented.colors[ii] = [0,1,0]
		draw_geometries([pcdSegmented])

		if k < 3.5:
			[k,idx,_] = pcdTree.search_knn_vector_3d(pcdSegmented.points[i],NofNN+1)
		print idx
		print pcdSegmented.points[idx[1]]
		nearbyPts = np.asarray(pcdSegmented.points)[idx[1:], :]
		nearbyPtsNormal = np.asarray(pcdSegmented.normals)[idx[1:], :]
		print nearbyPts
		print nearbyPtsNormal
		
		normalCurvatures = []
		## Define the local coordinates
		N = vectorops.unit(pcdSegmented.normals[i]) # also the "z"
		Y = vectorops.cross(N,[1,0,0]) # pq = vectorops.(q,p)
		X = vectorops.cross(Y,N)
		MMatrix=[]
		for j in range(k-1):
			#estimate the point normal, in Local corrdinate
			qGlobal = vectorops.sub(nearbyPts[j],pcdSegmented.points[i])
			qLocal = [vectorops.dot(qGlobal,X),vectorops.dot(qGlobal,Y),vectorops.dot(qGlobal,N)]		
			MGlobal = nearbyPtsNormal[j]
			MLocal = [vectorops.dot(MGlobal,X),vectorops.dot(MGlobal,Y),vectorops.dot(MGlobal,N)]
			[x_i,y_i,z_i] = qLocal
			[n_xi,n_yi,n_zi] = MLocal

			print x_i,n_xi,y_i,n_yi

			n_xy = (x_i*n_xi + y_i*n_yi)/math.sqrt(x_i*x_i+y_i*y_i)
			print n_xy
			k_i_n = - (n_xy)/(math.sqrt(n_xy*n_xy+n_zi*n_zi) + math.sqrt(x_i*x_i+y_i*y_i))
			normalCurvatures.append(k_i_n)

			## calculate the M matrix
			pQ = [qLocal[0],qLocal[1],0]
			angle = np.arccos(np.dot(X, pQ) / (np.linalg.norm(X) * np.linalg.norm(pQ)))
			#print 'MLocal: ',MLocal
			#print qLocal
			#print 'angle: ',angle
			row = [math.cos(angle)*math.cos(angle),2*math.cos(angle)*math.sin(angle),math.sin(angle)*math.sin(angle)]
			MMatrix.append(row)

		print "normalCurvatures", normalCurvatures	
		tmp = np.dot(np.transpose(MMatrix),MMatrix)
		tmp2 = np.dot(np.linalg.inv(tmp), np.transpose(MMatrix))
		x= tmp2.dot(normalCurvatures)
		print x
		eigenValues,v = np.linalg.eig([[x[0],x[1]],[x[1],x[2]]])
		curvatures.append(vectorops.norm_L1(eigenValues))
		#print vectorops.norm_L1(eigenValues)
		print 'progress' , float(i)/float(NofPts)


	print "max and min of curvatures: ",np.max(curvatures),np.min(curvatures)
	'''





	### ------------------------ Down sample the points for data collection --------
	pcdSegmentedDown= voxel_down_sample(pcdSegmented, voxel_size = 0.01)
	#NofPoints=countPts(pcdSegmentedDown)
	print 'N of Segmented Pts: ',len(pcdSegmentedDown.points)
	draw_geometries([pcdSegmentedDown])



	################### Need to save this point cloud ##################
	#each row should have position color normal
	pcdData=open('experiment_data/probePcd.txt','w')


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
	pcdData=open('experiment_data/originalPcd.txt','w')


	for (indPt,ptColor,ptNormal) in zip(pcdSegmented.points,pcdSegmented.colors,pcdSegmented.normals):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+'\n')
		#[k,idx,_] = pcdTree.search_knn_vector_3d(indPt,NofNN+1)
		#k = curvatures[idx[0]]
		#pcdData.write(str(k)+'\n')


	pcdData.close()

