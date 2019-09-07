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

def creat_sphere(radius,center,N):
    #create a sphere with approximately N surface points 
    #radius r, centered at center
    OPEN3DVIS = True
    surface_points = []
    Ncount = 0
    a = 4.0*math.pi*radius*radius/float(N)
    d = math.sqrt(a)
    Mtheta = round(math.pi/d)
    dv = math.pi/float(Mtheta)
    dphi = a/dv
    for i in range(int(Mtheta)):
        theta = math.pi*(float(i)+0.5)/Mtheta
        Mphi = round(2.0*math.pi*math.sin(theta)/dphi)
        for j in range(int(Mphi)-1):
            phi = 2.0*math.pi*float(j)/Mphi
            pt = [math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)]
            surface_points.append(vo.add(vo.mul(pt,radius),center))
            Ncount = Ncount + 1
    ##display for debugging    
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

def predict_sphere(pcd,param,discretization,num_iter,queryDList):
    DEBUGPROJECTEDPTS = False
    DEBUGDISPLACEDPTS = False
    OPEN3DVIS = False




    ################################
    #first create the 3D points for the circle
    #let the local x axis be -normal
    #local y's projection parallel with global y
    #then the local 2D points are on a y-z plane

    #first add points on the perimeter
    rigidPointsLocal = []
    Np = int(math.floor(diameter*math.pi/discretization))
    for i in range(Np):
        angle = float(i)/float(Np)*2.0*math.pi
        rigidPointsLocal.append(vo.mul([0,math.sin(angle),math.cos(angle)],diameter/2.0))
    #now generate a mesh
    Nm = int(math.ceil(diameter/2.0/discretization))
    leftUpper = [0,-float(Nm)*discretization,float(Nm)*discretization]
    rigidPointsLocal2 = []
    for i in range(2*Nm+1):
        for j in range(2*Nm+1):
            pt = vo.add(leftUpper,[0,float(i)*discretization,-float(j)*discretization])
            ##check if it is within discretization/2 from any of the perimeter points
            if vo.norm(pt) < diameter/2.0:
                if not (inRadius(pt,rigidPointsLocal,discretization/2.0)): 
                    rigidPointsLocal2.append(pt)

    rigidPointsLocal = rigidPointsLocal + rigidPointsLocal2
    #tmp = np.array(rigidPointsLocal)
    #plt.plot(tmp[:,1],tmp[:,2],'.')
    #plt.axis('equal')
    #plt.show()

    print 'There is a total of',len(rigidPointsLocal),'rigid surface pts'
    predictedForcesAll = []
    predictedTorquesAll = []
    #Iterate through different probe locations
    for pointNumber in num_iter:
        predictedForces = []
        predictedTorques = []
        #construct a local frame, centered on probedPcd
        center = probedPcd[pointNumber][0:3] #center in world frame
        circleNormal = probedPcd[pointNumber][6:9] # normal in world frame
        localXinW = vo.mul(circleNormal,-1)
        localYNotNormalized = [0,1,-localXinW[1]/localXinW[2]]
        localYinW = vo.div(localYNotNormalized,vo.norm(localYNotNormalized))
        localZinW = vo.cross(localXinW,localYinW)
        localZinW = vo.div(localZinW,vo.norm(localZinW))
        #rigidPointsLocal are already expressed in local coordinates...
        #now convert them to global coordinates when centered at probedPoints

        R = localXinW + localYinW + localZinW
        TlocalinW = (R,center)
        rigidPointsinW = []
        for ele in rigidPointsLocal:
            ptinW = se3.apply(TlocalinW,ele)
            rigidPointsinW.append(ptinW)

        ######## visualize in open3D for debugging ########
        if OPEN3DVIS:
            open3dCircularPcd = PointCloud()
            xyz = []
            rgb = []
            for ele in rigidPointsinW:
                xyz.append(ele[0:3])
                rgb.append([1,0,0])
            open3dCircularPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
            open3dCircularPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
            draw_geometries([open3dPcd,open3dCircularPcd])

        
        ################# project the pcd to the plane of the circle##############
        #The start point is the origin of the plane...
        projectedPcd = [] 
        #the projected Pcd is in local frame....
        projectedPcdIdxList = []#Idx in the original pcd 
        NofProjectedPoints = 0
        if OPEN3DVIS:
            pcdThatWasProjected = []
        for i in range(len(pcd)):
            p = pcd[i][0:3]
            projectedPt = vo.sub(p,vo.mul(circleNormal,vo.dot(vo.sub(p,center),circleNormal))) ##world origin
            projectedPt2D = vo.sub(projectedPt,center)#world origin
            projectedPt2DinLocal = [vo.dot(projectedPt2D,localYinW),vo.dot(projectedPt2D,localZinW)]
            #%make sure point is in the "box" defined by the line
            if vo.norm(projectedPt2DinLocal)< (diameter/2.0+0.001):
                NofProjectedPoints = NofProjectedPoints + 1
                projectedPcd.append(projectedPt2DinLocal)
                projectedPcdIdxList.append(i)
                if OPEN3DVIS:
                    pcdThatWasProjected.append(p)

        ######## visualize in open3D for debugging ########
        if OPEN3DVIS:
            open3dProjectedPcd = PointCloud()
            xyz = []
            rgb = []
            for ele in pcdThatWasProjected:
                xyz.append(ele[0:3])
                rgb.append([0,0,1])
            open3dProjectedPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
            open3dProjectedPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
            draw_geometries([open3dCircularPcd,open3dProjectedPcd])


        ###############Find the corresponding points on the deformable object############
        surfacePtsAll = []# %These are the surface pts that will be displaced.
        #Create a KDTree for searching
        projectedPcdTree = spatial.KDTree(projectedPcd)
        #average 3 neighbors
        NofN = 3   
        for i in range(len(rigidPointsinW)):
            ptLocal = rigidPointsLocal[i][1:3]
            d,Idx = projectedPcdTree.query(ptLocal,k=NofN)
            #We might end up having duplicated pts...
            #We should make sure that the discretization is not too fine..
            #or should average a few neighbors
            surfacePt = [0]*10
            for j in range(NofN):
                surfacePt = vo.add(surfacePt,pcd[projectedPcdIdxList[Idx[j]]][0:10])
            surfacePt = vo.div(surfacePt,NofN)
            surfacePtsAll.append(surfacePt)

        if OPEN3DVIS:
            open3dPcd1 = PointCloud()
            xyz = []
            rgb = []
            for ele in surfacePtsAll:
                xyz.append(ele[0:3])
                rgb.append([0,0,1])
            open3dPcd1.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
            open3dPcd1.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
            draw_geometries([open3dCircularPcd,open3dPcd1])
    
        ############# Go through a list of displacements
        #queryDList = [0.012]
        totalFinNList = []
        #for queryD = -0.003:0.001:0.014
        for queryD in queryDList:
            print 'queryD is',queryD
            ####calculate the nominal displacements
            surfacePts = []; #These are the surface pts that will be displaced...
            nominalD = []; #Column Vector..
            for i in range(len(rigidPointsinW)):
                circlePt = vo.sub(rigidPointsinW[i],vo.mul(circleNormal,queryD))
                surfacePt = surfacePtsAll[i][0:3]
                normal = surfacePtsAll[i][6:9]
                nominalDisp = -vo.dot(vo.sub(circlePt,surfacePt),normal)
                if nominalDisp > 0:
                    surfacePts.append(surfacePtsAll[i][0:10])#position in the global frame..
                    nominalD.append(nominalDisp)
            originalNominalD = deepcopy(nominalD)

            if OPEN3DVIS:
                open3dDisplacedPcd = PointCloud()
                xyz = []
                rgb = []
                for ele in surfacePts:
                    xyz.append(ele[0:3])
                    rgb.append([0,0,1])
                open3dDisplacedPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
                open3dDisplacedPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
                draw_geometries([open3dCircularPcd,open3dDisplacedPcd])
    
            #print('Deformed Surface Points',surfacePts)
            print('Calculating Actual D....')
            #####Calculate actual displacements
            NofSurfacePts = len(surfacePts)
            originalSurfacePts = deepcopy(surfacePts)
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
                        actualD = actualD.tolist()
                        positiveIndex = actualD >= 0
                        #surfacePts = surfacePts[positiveIndex]
                        surfacePts = [surfacePts[i] for i in range(len(surfacePts)) if actualD[i]>=0]
                        nominalD = [nominalD[i] for i in range(len(nominalD)) if actualD[i]>=0]
                    else:
                        negativeDisp = False

                ##########calculate force
                totalForce = 0
                #totalTorque = 0
                predictedForces.append(totalForce)
                #predictedTorques.append(totalTorque)

            else:
                predictedForces.append(0)
            
            if OPEN3DVIS:
                open3dPcd2 = PointCloud()
                xyz = []
                rgb = []
                for ele in surfacePts:
                    xyz.append(ele[0:3])
                    rgb.append([0,0,1])
                open3dPcd2.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
                open3dPcd2.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
                draw_geometries([open3dCircularPcd,open3dPcd2])
            #print originalNominalD
            #print nominalD,actualD
        predictedForcesAll.append(predictedForces)
    return predictedForcesAll

