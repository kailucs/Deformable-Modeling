# -*- coding: utf-8 -*-
from data_loader import *
from scipy import spatial
from copy import deepcopy
from klampt.math import se3,so3 
import matplotlib.pyplot as plt
import math
import time
from open3d import *

def invKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    return param/(param+r)

def inRadius(pt,pts,radius):
    for ele in pts:
        if vo.norm(vo.sub(ele,pt)) < radius:
            return True
    return False

def create_sphere(radius,center,N):
    #create a sphere with approximately N surface points 
    #radius r, centered at center
    #seems like the original algorithm has a bug...
    #In the second like instead of using r^2, just set it to be 1
    print(N)
    OPEN3DVIS = False
    surface_points = []
    Ncount = 0
    a = 4.0*math.pi/float(N)
    d = math.sqrt(a)
    Mtheta = round(math.pi/d)
    dv = math.pi/float(Mtheta)
    dphi = a/dv
    for i in range(int(Mtheta)):
        theta = math.pi*(float(i)+0.5)/Mtheta
        Mphi = round(2.0*math.pi*math.sin(theta)/dphi)
        for j in range(int(Mphi)):
            phi = 2.0*math.pi*float(j)/Mphi
            pt = [math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)]
            surface_points.append(vo.add(vo.mul(pt,radius),center))
            Ncount = Ncount + 1
    print('Ncount:',Ncount) 
    if OPEN3DVIS:
        open3dPcd = PointCloud()
        xyz = []
        rgb = []
        for ele in surface_points:
            xyz.append(ele[0:3])
            rgb.append([1,0,0])
        open3dPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
        draw_geometries([open3dPcd,open3dPcd])
        
    return surface_points 

def predict_sphere(pcd,probedPcd,rigidSurfacePtsAll,param,discretization,num_iter,queryDList,model,offset,deformableCenter,rigidCenter,diameter):
    DEBUGPROJECTEDPTS = False
    DEBUGDISPLACEDPTS = False
    OPEN3DVIS = False
    #create a pcd in open3D
    if OPEN3DVIS:
    #if False:
        open3dPcd = PointCloud()
        xyz = []
        rgb = []
        for ele in pcd:
            xyz.append(ele[0:3])
            rgb.append(ele[3:6])
        open3dPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))

    #rigidSurfacePtsAll are expressed locally, centered at rigidCenter
    #let the approach vector be local z and global y be local x
    approachVector = vo.sub(deformableCenter,rigidCenter)
    approachVector = vo.div(approachVector,vo.norm(approachVector))
    #approachVector = [0,0,-1]
    localXinW = [0,1,-approachVector[1]/approachVector[2]]
    localXinW = vo.div(localXinW,vo.norm(localXinW))
    localYinW = vo.cross(approachVector,localXinW)
    localYinW = vo.div(localYinW,vo.norm(localYinW))
    ##now we have both localX and localY normalized...

    ####compute rigid surface pts both locally and globally
    # To save time, do not include all the pts
    rigidPointsLocal = []# 2D local rigid pts
    rigidPointsinW = []
    for pt in rigidSurfacePtsAll:
        pt = vo.add(pt,rigidCenter)
        tmp = vo.sub(pt,rigidCenter)
        tmp = vo.div(tmp,vo.norm(tmp))
        if vo.dot(tmp,approachVector) > -0.2:
            projectedPt = vo.sub(pt,vo.mul(approachVector,vo.dot(vo.sub(pt,rigidCenter),approachVector))) ##world frame
            projectedPt2D = vo.sub(projectedPt,rigidCenter) #world frame
            projectedPt2DinLocal = [vo.dot(projectedPt2D,localXinW),vo.dot(projectedPt2D,localYinW)]
            rigidPointsLocal.append(projectedPt2DinLocal)
            rigidPointsinW.append(pt)

    #############Now Start predicting force#####################

    ######## visualize in open3D for debugging ########
    if OPEN3DVIS:
    #if True:
        open3dCircularPcd = PointCloud()
        xyz = []
        rgb = []
        for ele in rigidPointsinW:
            xyz.append(ele[0:3])
            rgb.append([1,0,0])
        open3dCircularPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dCircularPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
        draw_geometries([open3dPcd,open3dCircularPcd])

    
    ################# project the pcd to the local XY frame##############
    #The start point is the origin of the plane...
    projectedPcdinW = []
    projectedPcdLocal = [] 
    #the projected Pcd is in local frame....
    NofProjectedPoints = 0

    for i in range(len(pcd)):
        p = pcd[i][0:3]
        #### The way to select the point for the sphere is a bit different....
        if vo.norm(vo.sub(p,rigidCenter)) < (diameter/2.0+0.001):
            projectedPt = vo.sub(p,vo.mul(approachVector,vo.dot(vo.sub(p,rigidCenter),approachVector))) ##world origin
            projectedPt2D = vo.sub(projectedPt,rigidCenter)#world origin
            projectedPt2DinLocal = [vo.dot(projectedPt2D,localXinW),vo.dot(projectedPt2D,localYinW)]
            projectedPcdLocal.append(projectedPt2DinLocal)
            projectedPcdinW.append(pcd[i])
    if OPEN3DVIS:
        open3dPcd1 = PointCloud()
        xyz = []
        rgb = []
        for ele in projectedPcdinW:
            xyz.append(ele[0:3])
            rgb.append(ele[3:6])
        open3dPcd1.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd1.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
        draw_geometries([open3dCircularPcd,open3dPcd1])
    #print("number of projected deformable pts:",len(projectedPcdinW))
    ###############Find the corresponding points on the deformable object############
    surfacePtsAll = []# %These are the surface pts that will be displaced.
    rigidPointsFinal = []
    if len(projectedPcdinW) > 0:
        #Create a KDTree for searching
        projectedPcdTree = spatial.KDTree(projectedPcdLocal)
        #average 3 neighbors
        NofN = 3   
        for i in range(len(rigidPointsinW)):
            ptLocal = rigidPointsLocal[i][0:2]
            d,Idx = projectedPcdTree.query(ptLocal,k=NofN)
            #We might end up having duplicated pts...
            #We should make sure that the discretization is not too fine..
            #or should average a few neighbors
            if d[0] < 0.0025:
                surfacePt = [0]*10
                for j in range(NofN):
                    
                    surfacePt = vo.add(surfacePt,projectedPcdinW[Idx[j]][0:10])
                surfacePt = vo.div(surfacePt,NofN)
                surfacePtsAll.append(surfacePt)        
                rigidPointsFinal.append(rigidPointsinW[i][0:3])

        surfacePts = [] #These are the surface pts that will be displaced...
        nominalD = [] #Column Vector..
        rigidPtsInContact = []
        for i in range(len(rigidPointsFinal)):
            rigidPt = rigidPointsFinal[i]
            surfacePt = surfacePtsAll[i][0:3]
            normal = surfacePtsAll[i][6:9]
            nominalDisp = -vo.dot(vo.sub(rigidPt,surfacePt),normal)
            if nominalDisp > 0:
                surfacePts.append(surfacePtsAll[i][0:10])#position in the global frame..
                nominalD.append(nominalDisp)
                rigidPtsInContact.append(rigidPt)

        if OPEN3DVIS:
            open3dPcd1 = PointCloud()
            xyz = []
            rgb = []
            for ele in surfacePts:
                xyz.append(ele[0:3])
                rgb.append(ele[3:6])
            open3dPcd1.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
            open3dPcd1.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
            draw_geometries([open3dCircularPcd,open3dPcd1])

        originalNominalD = deepcopy(nominalD)
        #print('Deformed Surface Points',surfacePts)
        #####Calculate actual displacements
        NofSurfacePts = len(surfacePts)
        if NofSurfacePts > 0:
            negativeDisp = True
            while negativeDisp:
                NofSurfacePts = len(surfacePts)
                K = np.zeros((NofSurfacePts,NofSurfacePts))
                for i in range(NofSurfacePts):
                    for j in range(NofSurfacePts):
                        K[i][j] = invKernel(surfacePts[i][0:3],surfacePts[j][0:3],param)
                #print K
                actualD =  np.dot(np.linalg.inv(K),nominalD)
                #print nominalD,actualD
                negativeIndex = actualD < 0
                if np.sum(negativeIndex) > 0:
                    #print(len(surfacePts),len(nominalD),len(rigidPtsInContact))
                    actualD = actualD.tolist()
                    surfacePts = [surfacePts[i] for i in range(len(surfacePts)) if actualD[i]>=0]
                    nominalD = [nominalD[i] for i in range(len(nominalD)) if actualD[i]>=0]
                    rigidPtsInContact = [rigidPtsInContact[i] for i in range(len(rigidPtsInContact)) if actualD[i]>=0]
                else:
                    negativeDisp = False

            ##########calculate force and torque
            totalForce = 0
            Ns = len(surfacePts)
            queryPtsBeforeNormalization = []
            for i in range(Ns):
                queryPt = surfacePts[i][0:3] + [nominalD[i]-actualD[i]]
                queryPtsBeforeNormalization.append(queryPt)
                #queryPts.append(queryPt)
            for i in range(Ns):
                queryPt = surfacePts[i][0:3] + [nominalD[i]]
                queryPtsBeforeNormalization.append(queryPt)

                
            queryPts = normalize_points(np.array(queryPtsBeforeNormalization),offset[0:3],offset[3])    
            forces = model.predict(queryPts)
            #print(forces)
            for i in range(Ns): 
                force = forces[i+Ns]-forces[i]
                if force < 0:
                    force = 0
                totalForce = totalForce + force
        
        return totalForce
    return 0

