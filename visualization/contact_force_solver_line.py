from data_loader import *
from scipy import spatial
from copy import deepcopy
import time
from open3d import *
import open3d
import matplotlib.pyplot as plt
import time
from klampt.math import vectorops as vo
import colorsys
def invKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    return param/(param+r)
def linearKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    if r <= param:
        return (param-r)/param
    else:
        return 0 

def debug(vis,world):
    vis.show()
    startTime = time.time()
    probe = world.rigidObject(0)
    klampt_pcd_object = world.rigidObject(1) #this is a rigid object
    klampt_pcd = klampt_pcd_object.geometry().getPointCloud() #this is now a PointCloud()
    N_pts = klampt_pcd.numPoints()
    for i in klampt_pcd.propertyNames:
        print(i)
    while vis.shown() and (time.time() - startTime < 10):
        vis.lock()
        for i in range(N_pts):
            klampt_pcd.setProperty(i,'r',np.random.rand())
            klampt_pcd.setProperty(i,'g',np.random.rand())
            klampt_pcd.setProperty(i,'b',np.random.rand())
            klampt_pcd.setPoint(i,[np.random.rand(),np.random.rand(),np.random.rand()])
        klampt_pcd_object.geometry().setPointCloud(klampt_pcd)
        vis.unlock()
        #print(time.time() - startTime)
        time.sleep(0.05)
    vis.kill()
    return

def predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,vis,world):
    vision_task_1 = False#True means we are doing the collision detection.
    o3d = False
    #create a pcd in open3D
    equilibriumPcd = open3d.geometry.PointCloud()
    xyz = []
    rgb = []
    normals = []
    for ele in pcd:
        xyz.append(ele[0:3])
        rgb.append(ele[3:6])
        normals.append(ele[6:9])
    equilibriumPcd.points = open3d.utility.Vector3dVector(np.asarray(xyz,dtype=np.float32))
    equilibriumPcd.colors = open3d.utility.Vector3dVector(np.asarray(rgb,dtype=np.float32))
    equilibriumPcd.normals = open3d.utility.Vector3dVector(np.asarray(normals,dtype=np.float32))
    # equilibriumPcd,ind=statistical_outlier_removal(equilibriumPcd,nb_neighbors=20,std_ratio=0.75) 
    if o3d:
        vis3 = open3d.visualization.Visualizer()
        vis3.create_window()
        open3dPcd = open3d.geometry.PointCloud()
        vis3.add_geometry(open3dPcd)

    probe = world.rigidObject(0)
    klampt_pcd_object = world.rigidObject(1) #this is a rigid object
    klampt_pcd = klampt_pcd_object.geometry().getPointCloud() #this is now a PointCloud()
    N_pts = klampt_pcd.numPoints()
    dt = 0.1
   

    for pointNumber in num_iter:

        ####Preprocess the pcd ####
        lineStart0 = lineStarts[pointNumber]
        lineEnd0 =lineEnds[pointNumber]
        lineTorqueAxis = lineTorqueAxes[pointNumber]
        N = 1 + round(vo.norm(vo.sub(lineStart0,lineEnd0))/discretization)
        s = np.linspace(0,1,N)
        lineNormal = lineNormals[pointNumber]
        localXinW = vo.sub(lineEnd0,lineStart0)/np.linalg.norm(vo.sub(lineEnd0,lineStart0))
        localYinW = (lineTorqueAxis)/np.linalg.norm(lineTorqueAxis)
        lineStartinLocal = [0,0]
        lineEndinLocal = [np.dot(vo.sub(lineEnd0,lineStart0),localXinW),
                        np.dot(vo.sub(lineEnd0,lineStart0),localYinW)]
        #################first project the pcd to the plane of the line##############
        #The start point is the origin of the plane...
        projectedPcd = [] #Each point is R^10 
        projectedPcdLocal = [] #localXY coordinate
        #the projected Pcd is in local frame....
        for i in range(len(pcd)):
            p = pcd[i][0:3]
            projectedPt = vo.sub(p,vo.mul(lineNormal,vo.dot(vo.sub(p,lineStart0),lineNormal))) ##world origin
            projectedPt2D = vo.sub(projectedPt,lineStart0)#world origin
            projectedPt2DinLocal = [vo.dot(projectedPt2D,localXinW),vo.dot(projectedPt2D,localYinW)]
            #%make sure point is in the "box" defined by the line
            if ((projectedPt2DinLocal[0]<0.051) and (projectedPt2DinLocal[0] > -0.001)
                and (projectedPt2DinLocal[1]<0.001) and (projectedPt2DinLocal[1]>-0.001)):
                projectedPcdLocal.append(projectedPt2DinLocal)
                projectedPcd.append(pcd[i])
        #Create a KDTree for searching
        projectedPcdTree = spatial.KDTree(projectedPcdLocal)
        ###############Find the corresponding point on the surface to the line############
        surfacePtsAll = []# %These are the surface pts that will be displaced.
        #%part of the probe not in contact with the object...
        #average 3 neighbors
        NofN = 3
        #rigidPointsFinal = []   ##we have a potential bug here for not using rigidPointsFinal....      
        for i in range(int(N)):
            tmp =vo.mul(vo.sub(lineEnd0,lineStart0),s[i])#%the point on the line, projected 
            linePt = [vo.dot(tmp,localXinW),vo.dot(tmp,localYinW)]
            d,Idx = projectedPcdTree.query(linePt[0:2],k=NofN)
            #We might end up having duplicated pts...
            #We should make sure that the discretization is not too fine..
            #or should average a few neighbors
            if d[0] < 0.002:
                surfacePt = [0]*10
                for j in range(NofN):
                    surfacePt = vo.add(surfacePt,projectedPcd[Idx[j]][0:10])
                surfacePt = vo.div(surfacePt,NofN)
                surfacePtsAll.append(surfacePt) #position in the global frame..
                #rigidPointsFinal.append(linePts)

        N = len(surfacePtsAll)

        ##### Preprocesss done
        if not o3d:
            vis.show()
            time.sleep(15.0)
        ############# Go through a list of displacements
        totalFinNList = []
        #for queryD = -0.003:0.001:0.014
        for queryD in queryDList:
            lineStart = vo.sub(lineStart0,vo.mul(lineNormal,queryD))
            lineEnd = vo.sub(lineEnd0,vo.mul(lineNormal,queryD))
            lineCenter = vo.div(vo.add(lineStart,lineEnd),2)
            torqueCenter = vo.add(lineCenter,vo.mul(lineNormal,0.09))


            if vision_task_1:

                vis.lock()
                print(queryD)
                x_axis = vo.mul(vo.unit(vo.sub(lineEnd,lineStart)),-1.0)
                y_axis = [lineTorqueAxis[0],lineTorqueAxis[1],lineTorqueAxis[2]]
                z_axis = vo.cross(x_axis,y_axis)
                z_axis = [z_axis[0],z_axis[1],z_axis[2]]
                R = x_axis+y_axis+z_axis
                t = vo.add(lineCenter,vo.mul(lineNormal,0.002))
                
                print(R,t)
                probe.setTransform(R,t)
                vis.unlock()
                time.sleep(dt)
            else:
                if not o3d:
                    vis.lock()
                print(queryD)
                x_axis = vo.mul(vo.unit(vo.sub(lineEnd,lineStart)),-1.0)
                y_axis = [lineTorqueAxis[0],lineTorqueAxis[1],lineTorqueAxis[2]]
                z_axis = vo.cross(x_axis,y_axis)
                z_axis = [z_axis[0],z_axis[1],z_axis[2]]
                R = x_axis+y_axis+z_axis
                t = vo.add(lineCenter,vo.mul(lineNormal,0.003))
                probe.setTransform(R,t)
                

                ####calculate the nominal displacements
                surfacePts = [] #These are the surface pts that will be displaced...
                rigidPtsInContact = []
                nominalD = [] #Column Vector..
                for i in range(int(N)): ##we have a potential bug here for keep interating trhough N, since about, if d[0] < 0.002, the points is not added...
                    #weird no bug occured...
                    linePt = vo.add(vo.mul(vo.sub(lineEnd,lineStart),s[i]),lineStart)              
                    surfacePt = surfacePtsAll[i][0:3]
                    normal = surfacePtsAll[i][6:9]
                    nominalDisp = -vo.dot(vo.sub(linePt,surfacePt),normal)
                    if nominalDisp > 0:
                        surfacePts.append(surfacePtsAll[i][0:10])#position in the global frame..
                        rigidPtsInContact.append(linePt)
                        nominalD.append(nominalDisp)
                originalNominalD = deepcopy(nominalD)
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
                        #startTime = time.time()
                        actualD =  np.dot(np.linalg.inv(K),nominalD)
                        #print('Time spent inverting matrix',time.time()- startTime)
                        #print nominalD,actualD
                        negativeIndex = actualD < 0
                        if np.sum(negativeIndex) > 0:
                            actualD = actualD.tolist()
                            surfacePts = [surfacePts[i] for i in range(len(surfacePts)) if actualD[i]>=0]
                            nominalD = [nominalD[i] for i in range(len(nominalD)) if actualD[i]>=0]
                            rigidPtsInContact = [rigidPtsInContact[i] for i in range(len(rigidPtsInContact)) if actualD[i]>=0]
                        else:
                            negativeDisp = False
                    
                    
                    #hsv's  hue: 0.25 = 0.0002 displacement 1 == 10 displacement
                    #generate the entire displacement field
                    #surfacePts are the equlibrium pts that provide 
                    entireDisplacedPcd = []
                    colorThreshold = 0.0005
                    colorUpperBound = 0.012
                    forceColorUpperBound = 0.004
                    colorRange = colorUpperBound - colorThreshold
                    greyColor = 0.38
                    
                    xyz = []
                    rgb = []
                    xyzForce = []
                    rgbForce = []
                    maxDisp = 0
                    for pt,ptNormal in zip(equilibriumPcd.points,equilibriumPcd.normals):
                        shapeVector = []
                        for pt2 in surfacePts:
                            shapeVector.append(invKernel(pt[0:3],pt2[0:3],param))
                        nominalDisplacement = vo.dot(shapeVector,actualD)
                        if nominalDisplacement > maxDisp:
                            maxDisp = nominalDisplacement

                        color = colorsys.hsv_to_rgb(nominalDisplacement/colorRange*0.75+0.25,1,0.75)
                        color = [color[0],color[1],color[2]]
                        displacedDeformablePoint = vo.sub(pt[0:3],vo.mul(ptNormal,nominalDisplacement))
                        xyz.append(displacedDeformablePoint)
                        rgb.append(color)
                        

                    counter = 0 
                    for (p,c) in zip(xyz,equilibriumPcd.colors):#rgb):
                        klampt_pcd.setProperty(counter,'r',c[0])
                        klampt_pcd.setProperty(counter,'g',c[1])
                        klampt_pcd.setProperty(counter,'b',c[2])
                        klampt_pcd.setPoint(counter,p)
                        counter = counter + 1
                    klampt_pcd_object.geometry().setPointCloud(klampt_pcd)
                    if o3d:
                        open3dPcd.points = open3d.utility.Vector3dVector(np.asarray(xyz,dtype=np.float32))
                        open3dPcd.colors = open3d.utility.Vector3dVector(np.asarray(rgb,dtype=np.float32))
                        open3dPcd.estimate_normals(search_param = open3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
                        ###open3dPcd,ind=statistical_outlier_removal(open3dPcd,nb_neighbors=10,std_ratio=2) 
                        #open3d.visualization.draw_geometries([open3dPcd])#_box_bottom])
                        vis3.add_geometry(open3dPcd)
                        vis3.poll_events()
                        vis3.update_renderer()
                    if not o3d:
                        vis.unlock()
                    time.sleep(dt)
                else:
                    pass

        
        while vis.shown():
            time.sleep(0.1)
    return

