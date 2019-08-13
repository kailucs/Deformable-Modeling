import numpy as np
from klampt import *
from klampt.math import vectorops,so3,se3
import math
from open3d import *

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
    colors=[]
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
        colors.append(l2[3:6])
		normals.append(l2[6:9])
		if pcdtype == 'xyzrgbntheta':
			normal_theta.append(l2[10:13])
			theta.append(l2[13])
			pt_index.append(l2[14])
	dataFile.close()
	print '---------------------pcd loaded -----------------------------'
	if pcdtype == 'xyzrgbn':
		#return points, normals
        tmp = PointCloud()
        tmp.points = Vector3dVector(np.array(points))
	    tmp.colors = Vector3dVector(np.array(colors))
	    tmp.normals = Vector3dVector(np.array(normals))
        return tmp

	elif pcdtype == 'xyzrgbntheta':
		return points, normals, normal_theta, theta, pt_index

def main():
    oripcd_path = '/probePcd.txt'
    probepcd_path = '/originalPcd.txt'

    oripcd = load_pcd(oripcd_path)
    probepcd = load_pcd(probepcd_path)

    ori_curvatures = cal_curvature(oripcd)

    xyzrgb_oripcd = np.hstack((np.array(oripcd.points), np.array(oripcd.colors)))
    xyzrgb_probepcd = np.hstack((np.array(probepcd.points), np.array(probepcd.colors)))

    tmp_xyzrgb_oripcd = [tuple(row) for row in xyzrgb_oripcd]
    tmp_xyzrgb_probepcd = [tuple(row) for row in xyzrgb_probepcd]

    probe_curvatures = []

    for point in tmp_xyzrgb_probepcd:
        index = tmp_xyzrgb_oripcd.index(point)
        probe_curvature = ori_curvatures[index]
        probe_curvatures.append(probe_curvature)
    
    pcdData=open('/probePcd_cur.txt','w')
	for (indPt,ptColor,ptNormal,cur) in zip(probepcd.points,probepcd.colors,probepcd.normals,probe_curvatures):
		pcdData.write(str(indPt[0])+' '+str(indPt[1])+' '+str(indPt[2])+' ')
		pcdData.write(str(ptColor[0])+' '+str(ptColor[1])+' '+str(ptColor[2])+' ')
		pcdData.write(str(ptNormal[0])+' '+str(ptNormal[1])+' '+str(ptNormal[2])+' ')
		pcdData.write(str(cur[0])+'\n')
	pcdData.close()

if __name__ == "__main__":
    main()