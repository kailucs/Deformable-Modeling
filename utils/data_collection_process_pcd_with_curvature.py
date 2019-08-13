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
from utils.voxelgrid_filter import voxelgrid_downsample

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
	# read pcd
	drawFlag = config.drawFlag
	path_ori_pcd = config.exp_path+'exp_'+str(config.exp_number)+"/TSDF_converted.ply"
	originalPcd = read_point_cloud(path_ori_pcd,format='ply')

	if drawFlag:
		draw_geometries([originalPcd])

	# remove the duplicates
	originalPcd_rmd = PointCloud()
	print('[*]Befor duplicate rm: %d'%len(originalPcd.points))
	tmp_xyz = np.array(originalPcd.points)
	tmp_rgb = np.array(originalPcd.colors)

	tmp_xyzrgb = np.hstack((tmp_xyz,tmp_rgb))
	tmp_xyzrgb = [tuple(row) for row in tmp_xyzrgb]
	set_xyzrgb = list(set(tmp_xyzrgb))

	set_xyzrgb = np.array(set_xyzrgb)
	xyz_array = set_xyzrgb[:,0:3]
	rgb_array = set_xyzrgb[:,3:6]

	originalPcd_rmd.points = Vector3dVector(xyz_array)
	originalPcd_rmd.colors = Vector3dVector(rgb_array)	
	print('[*]After duplicate rm: %d'%len(originalPcd_rmd.points))
	
	if drawFlag:
		draw_geometries([originalPcd_rmd])

	# remove outlier
	originalPcd_rmd,ind=statistical_outlier_removal(originalPcd_rmd,nb_neighbors=40,std_ratio=0.8)
	print("[*]After outliers rm: %d"%len(originalPcd_rmd.points))
	NofPtsOriginal = len(originalPcd_rmd.points)

	if drawFlag:
		draw_geometries([originalPcd_rmd])
	
	#TODO:
	originalPcd_rmd = voxelgrid_downsample(originalPcd_rmd,[0.00075,0.00075,0.001],type='zmin',with_normals=False) #TODO: exp0 using 0.015 
	#originalPcd_rmd = voxelgrid_downsample(originalPcd_rmd,[0.002,0.002,0.002],type='zmin',with_normals=False)
	print '[*]After VoxelGrid Filter(Type: middle): ',len(originalPcd_rmd.points)

	# estimate normals (using 30 NN)
	estimate_normals(originalPcd_rmd, search_param = KDTreeSearchParamHybrid(radius = 0.03, max_nn = 60)) # normals can actually point downwards......
		
	# fix normals
	n = np.array(originalPcd_rmd.normals)
	n[n[:,2]<0] = - n[n[:,2]<0] #fix normals with z axis
	originalPcd_rmd.normals = Vector3dVector(n)

	draw_geometries([originalPcd_rmd])
	
	# curvature
	if config.use_curvature == True:
		curvature = cal_curvature(originalPcd_rmd)
	else:
		curvature = np.zeros((len(originalPcd_rmd.points),1))

	pcdData=open(config.exp_path+'exp_'+str(config.exp_number)+'/originalPcd.txt','w')
	for (indPt,ptColor,ptNormal,cur) in zip(originalPcd_rmd.points,originalPcd_rmd.colors,originalPcd_rmd.normals,curvature):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+' ')
		pcdData.write(str(cur[0])+'\n')
	pcdData.close()
	
	# pcd segmented	
	pcdSegmented = PointCloud()
	xyz = np.array(originalPcd_rmd.points)
	rgb = np.array(originalPcd_rmd.colors)
	xyzrgbn = np.hstack((xyz,rgb,n))
	xyzrgbn = xyzrgbn[xyzrgbn[:,8]>0.8] #rm some flat points
	
	xyzrgbn = xyzrgbn[xyzrgbn[:,1]>0.00]#TODO: in exp0, set y >0.03, to cut its foot. record next time in log file
	xyzrgbn = xyzrgbn[xyzrgbn[:,1]<0.12]
	xyzrgbn = xyzrgbn[xyzrgbn[:,0]>-0.52]
	xyzrgbn = xyzrgbn[xyzrgbn[:,0]<-0.41]

	xyzrgbn = xyzrgbn[xyzrgbn[:,2]>0.89]
	#print(xyzrgbn[:,2])
	#for poking, need points away from boundary
	#center = [-0.50,0.075]
	#xyzrgbn = xyzrgbn[((xyzrgbn[:,0]-center[0])**2+(xyzrgbn[:,1]-center[1])**2)**0.5<0.07]

	pcdSegmented.points = Vector3dVector(xyzrgbn[:,0:3])
	pcdSegmented.colors = Vector3dVector(xyzrgbn[:,3:6])
	pcdSegmented.normals = Vector3dVector(xyzrgbn[:,6:9])

	draw_geometries([pcdSegmented])

	# Down sample the points for data collection, using voxelgrid filter
	pcdSegmentedDown = voxelgrid_downsample(pcdSegmented,[0.013,0.013,0.013]) #TODO: exp0 using 0.015 
	print '[*]After VoxelGrid Filter(Type: middle): ',len(pcdSegmentedDown.points)
	draw_geometries([pcdSegmentedDown])

	# save pcd
	pcdData=open(config.exp_path+'exp_'+str(config.exp_number)+'/probePcd.txt','w')
	for (indPt,ptColor,ptNormal,cur) in zip(pcdSegmentedDown.points,pcdSegmentedDown.colors,pcdSegmentedDown.normals,curvature):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+' ')
		pcdData.write(str(cur[0])+'\n')
	pcdData.close()

	
	print('[*]Processing PCD Done.')

def deleteDup(list1):
	res = []
	for item in list1:
		if not item in res:
			res.append(item)
	return res

def cal_curvature(pcdSegmented):
	NofPts=len(pcdSegmented.points)
	NofNN=30
	searchR=0.01  #the points are 2mm apart...

	pcdTree = KDTreeFlann(pcdSegmented)
	curvatures=[]

	print 'calculating curvatures......'
	for i in range(len(pcdSegmented.points)):
	#for i in [0]:
		[k,idx,_] = pcdTree.search_knn_vector_3d(pcdSegmented.points[i],NofNN+1)
		#[k,idx,_] = pcdTree.search_radius_vector_3d(pcdSegmented.points[i],searchR)
		#k is the total Number, while idx is the list of indices
		#for debugging
		#for ii in idx:
		#	pcdSegmente.colors[ii] = [0,1,0]
		#draw_geometries([pcdSegmented])

		#if k < 3.5:
		#	[k,idx,_] = pcdTree.search_knn_vector_3d(pcdSegmented.points[i],NofNN+1)
		nearbyPts = np.asarray(pcdSegmented.points)[idx[1:], :]
		nearbyPtsNormal = np.asarray(pcdSegmented.normals)[idx[1:], :]

		
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

			#print x_i,n_xi,y_i,n_yi
			n_xy = (x_i*n_xi + y_i*n_yi)/math.sqrt(x_i*x_i+y_i*y_i)
			#print n_xy
			k_i_n = - (n_xy)/(math.sqrt(n_xy*n_xy+n_zi*n_zi) + math.sqrt(x_i*x_i+y_i*y_i))
			normalCurvatures.append(k_i_n)

			## calculate the M matrix
			pQ = [qLocal[0],qLocal[1],0]
			angle = np.arccos(np.dot(X, pQ) / (np.linalg.norm(X) * np.linalg.norm(pQ)))
			row = [math.cos(angle)*math.cos(angle),2*math.cos(angle)*math.sin(angle),math.sin(angle)*math.sin(angle)]
			MMatrix.append(row)
		#perform least squres fitting
		tmp = np.dot(np.transpose(MMatrix),MMatrix)
		tmp2 = np.dot(np.linalg.inv(tmp), np.transpose(MMatrix))
		x= tmp2.dot(normalCurvatures)
		eigenValues,v = np.linalg.eig([[x[0],x[1]],[x[1],x[2]]])
		curvatures.append(vectorops.norm_L1(eigenValues))
		#print vectorops.norm_L1(eigenValues)
		print 'progress' , float(i)/float(NofPts)

	print "max and min of curvatures: ",np.max(curvatures),np.min(curvatures)
	return curvatures

def load_pcd(path, pcdtype='xyzrgbn'):
	points=[]
	normals=[]
	colors=[]
	normal_theta=[]
	theta=[]
	pt_index=[]
	dataFile=open(path,'r')
	for line in dataFile:
		line=line.rstrip()
		l=[num for num in line.split(' ')]
		l2=[float(num) for num in l]
		points.append(l2[0:3])
		colors.append(l2[3:6])
		normals.append(l2[6:9])
		if pcdtype == 'xyzrgbntheta':
			normal_theta.append(l2[10:13])
			theta.append(l2[13])
			pt_index.append(l2[14])
	dataFile.close()
	print '---------------------pcd loaded -----------------------------'
	if pcdtype == 'xyzrgbn':
		return points, colors, normals
	elif pcdtype == 'xyzrgbntheta':
		return points, normals, normal_theta, theta, pt_index

	'''
	#read pcd
	pcds = []
	for i in range(5):
		tmp_path = config.exp_path+'exp_'+str(config.exp_number)+"/processed/objectScan_"+str(i)+".pcd"
		tmp_pcd = read_point_cloud(tmp_path,format='pcd')
		pcds.append(tmp_pcd)

	# select
	tmp_select_pcd = PointCloud()
	for i in range(0,5):
		tmp_xyz = np.array(pcds[i].points)
		tmp_rgb = np.array(pcds[i].colors)
		tmp_xyzrgb = np.hstack((tmp_xyz,tmp_rgb))

		draw_geometries([pcds[i]])

		if i == 0:
			merge_xyzrgb_array = tmp_xyzrgb
		
		else:
						
			#center = [-0.50,0.105]
			#tmp_xyzrgb = tmp_xyzrgb[((tmp_xyzrgb[:,0]-center[0])**2+(tmp_xyzrgb[:,1]-center[1])**2)**0.5<0.02]
			
			tmp_xyzrgb = tmp_xyzrgb[tmp_xyzrgb[:,2]<0.976]
			tmp_xyzrgb = tmp_xyzrgb[tmp_xyzrgb[:,2]>0.968]
			merge_xyzrgb_array = np.vstack((merge_xyzrgb_array,tmp_xyzrgb))

			tmp_select_pcd.points = Vector3dVector(tmp_xyzrgb[:,0:3])
			tmp_select_pcd.colors = Vector3dVector(tmp_xyzrgb[:,3:6])

			draw_geometries([tmp_select_pcd])

	# remove the duplicates
	final_merge_pcd = PointCloud()

	print('[*]Befor duplicate rm: %d'%merge_xyzrgb_array.shape[0])

	merge_xyzrgb_array = [tuple(row) for row in merge_xyzrgb_array]
	merge_xyzrgb_array = list(set(merge_xyzrgb_array))

	merge_xyzrgb_array = np.array(merge_xyzrgb_array)
	xyz_array = merge_xyzrgb_array[:,0:3]
	rgb_array = merge_xyzrgb_array[:,3:6]

	final_merge_pcd.points = Vector3dVector(xyz_array)
	final_merge_pcd.colors = Vector3dVector(rgb_array)
	final_merge_pcd.normals = Vector3dVector(np.tile(np.array([0,0,1]),(xyz_array.shape[0],1)))	
	print('[*]After duplicate rm: %d'%merge_xyzrgb_array.shape[0])
	
	draw_geometries([final_merge_pcd])

	final_merge_pcd_down = voxelgrid_downsample(final_merge_pcd,[0.001,0.001,0.05],type='special') #TODO: exp0 using 0.015 
	print '[*]After VoxelGrid Filter(Type: middle): ',len(final_merge_pcd_down.points)
	draw_geometries([final_merge_pcd_down])

	exit()
	'''
