from data_loader import *
from scipy import spatial
from copy import deepcopy
import time
from open3d import *
import matplotlib.pyplot as plt
def invKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    return param/(param+r)
def linearKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    if r <= param:
        return (param-r)/param
    else:
        return 0 

def predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset):
    DEBUGPROJECTEDPTS = False
    DEBUGDISPLACEDPTS = False
    OPEN3DVIS = False
    #create a pcd in open3D
    if OPEN3DVIS:
        open3dPcd = PointCloud()
        xyz = []
        rgb = []
        for ele in pcd:
            xyz.append(ele[0:3])
            #rgb.append(ele[3:6])
            rgb.append([0.1,0.1,0.1])
        open3dPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))

    predictedForcesAll = []
    predictedTorquesAll = []
    for pointNumber in num_iter:
        predictedForces = []
        predictedTorques = []
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


        if OPEN3DVIS:
            open3dDisplacedPcd = PointCloud()
            xyz = []
            rgb = []
            for ele in surfacePtsAll:
                xyz.append(ele[0:3])
                rgb.append([0,0,1])
            open3dDisplacedPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
            open3dDisplacedPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
            draw_geometries([open3dPcd,open3dDisplacedPcd])

        N = len(surfacePtsAll)
        ############# Go through a list of displacements
        totalFinNList = []
        #for queryD = -0.003:0.001:0.014
        for queryD in queryDList:
            #startTime2 = time.time()
            #print(queryD)
            lineStart = vo.sub(lineStart0,vo.mul(lineNormal,queryD))
            lineEnd = vo.sub(lineEnd0,vo.mul(lineNormal,queryD))
            lineCenter = vo.div(vo.add(lineStart,lineEnd),2)
            torqueCenter = vo.add(lineCenter,vo.mul(lineNormal,0.09))
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
            ###remove duplicates
            #urfacePts = list(set(surfacePts))
            #rigidPtsInContact = list(set(rigidPtsInContact))
            #nominalD = list(set(nominal))
            #print(surfacePts)
            '''
            if OPEN3DVIS:

                open3dRigidPcd = PointCloud()
                xyz = []
                rgb = []
                for ele in rigidPtsInContact:
                    xyz.append(ele[0:3])
                    rgb.append([1,0,0])
                open3dRigidPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
                open3dRigidPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))

                open3dDisplacedPcd = PointCloud()
                xyz = []
                rgb = []
                for ele in surfacePts:
                    xyz.append(ele[0:3])
                    rgb.append([0,0,1])
                open3dDisplacedPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
                open3dDisplacedPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
                draw_geometries([open3dPcd,open3dDisplacedPcd,open3dRigidPcd]
            '''
            #print('Deformed Surface Points',surfacePts)
            #print('Calculating Actual D....')
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
                
                ##########calculate force and torque
                totalForce = 0
                totalTorque = 0

                #predictedTorques.append(totalTorque)
                #print("actual displacement:",actualD)
                #startTime = time.time()
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
                #print(queryPts)
                #print(forces)
                for i in range(Ns): 
                    force = forces[i+Ns]-forces[i]
                    if force<0:
                        force = 0
                    totalForce = totalForce + force
                    torqueArm = vo.sub(rigidPtsInContact[i],torqueCenter)
                    normal = surfacePts[i][6:9]
                    normal = vo.div(normal,vo.norm(normal))
                    torque = vo.cross(torqueArm,vo.mul(normal,force))
                    totalTorque = totalTorque + vo.dot(torque,lineTorqueAxis)  
                #timeSpentQueryingModel = time.time()- startTime
                #print('Time spent querying point model',timeSpentQueryingModel)
                
                predictedForces.append(totalForce)
                predictedTorques.append(totalTorque)
                #print('forces',forces)
                #print('nominalD:',nominalD)
                #print("actualD:",actualD)
            else:
                predictedForces.append(0)
                predictedTorques.append(0)
            #print originalNominalD
            
            #print('nominalD:',nominalD)
            #print("actualD:",actualD)          
            #timeSpentQueryingDisplacement = time.time()- startTime2
            #print('Time spent querying a displacement',timeSpentQueryingDisplacement)
            #if NofSurfacePts > 0:
            #    print('Ratio:',timeSpentQueryingModel/timeSpentQueryingDisplacement)
        #print("actual displacemetn",actualD)
        #plt.plot(queryDList,predictedForces)
        #plt.show()
        predictedForcesAll.append(predictedForces)
        predictedTorquesAll.append(predictedTorques)
    return predictedForcesAll,predictedTorquesAll

