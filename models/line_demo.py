from data_loader import *
from scipy import spatial
from copy import deepcopy
import time
from open3d import *
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
import math
from copy import deepcopy
#from openmesh import *
import colorsys
params = [0.03,0.15,0.02,0.01,0.008]

def invKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    return param/(param+r)
def linearKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    if r <= param:
        return (param-r)/param
    else:
        return 0 
def inList(ele,alist):
    counter = 0
    for ele2 in alist:
        if vo.norm(vo.sub(ele[0:3],ele2[0:3])) < 0.0014:
            return (True,counter)
        else:
            counter = counter + 1
    return (False,counter)
def predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset,p_min,p_max):
    DEBUGPROJECTEDPTS = False
    DEBUGDISPLACEDPTS = False
    OPEN3DVIS = False
    FIRSTPIC = 1



    #create a pcd in open3D
    equilibriumPcd = PointCloud()
    xyz = []
    rgb = []
    normals = []
    for ele in pcd:
        xyz.append(ele[0:3])
        rgb.append(ele[3:6])
        normals.append(ele[6:9])
    equilibriumPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
    equilibriumPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
    equilibriumPcd.normals = Vector3dVector(np.asarray(normals,dtype=np.float32))
    equilibriumPcd,ind=statistical_outlier_removal(equilibriumPcd,nb_neighbors=20,std_ratio=0.75) 

    ## Add a bottom..
    sideLength = 0.5
    plateCenter = [sideLength/2.0,sideLength/2.0]
    mesh_box_bottom = geometry.create_mesh_box(sideLength,sideLength,0.005) #x,y,z
    mesh_box_bottom.paint_uniform_color([0.38,0.38,0.38])
    transform = np.asarray(
                [[1, 0, 0,  (p_max[0]-p_min[0])/2.0+p_min[0]-plateCenter[0]],
                [0, 1, 0,  (p_max[1]-p_min[1])/2.0+p_min[1]-plateCenter[1]],
                [0, 0, 1, p_min[2]],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_bottom.transform(transform)    
    mesh_box_bottom.compute_vertex_normals()



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
        if FIRSTPIC:
            pass
            queryDList = [-0.09]

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
            for i in range(int(N)):
                linePt = vo.add(vo.mul(vo.sub(lineEnd,lineStart),s[i]),lineStart)              
                surfacePt = surfacePtsAll[i][0:3]
                normal = surfacePtsAll[i][6:9]
                nominalDisp = -vo.dot(vo.sub(linePt,surfacePt),normal)
                if nominalDisp > 0:
                    surfacePts.append(surfacePtsAll[i][0:10])#position in the global frame..
                    rigidPtsInContact.append(linePt)
                    nominalD.append(nominalDisp)
            originalNominalD = deepcopy(nominalD)
            if FIRSTPIC:
                ##load probe
                probe_mesh = read_triangle_mesh('Contact_Probe_linear.ply')
                probe_mesh.compute_vertex_normals()
                probeCenter = deepcopy(lineCenter)
                probeX = deepcopy(localXinW)
                probeY = deepcopy(localYinW)
                probeZ = vo.cross(probeX,probeY)
                transform = np.asarray(
                [[probeX[0], probeY[0], probeZ[0],  probeCenter[0]],
                [probeX[1], probeY[1], probeZ[1],  probeCenter[1]],
                [probeX[2], probeY[2], probeZ[2], probeCenter[2]],
                [0.0, 0.0, 0.0, 1.0]])
                probe_mesh.transform(transform) 
                probe_mesh.paint_uniform_color([0.8,0,0])
                draw_geometries([equilibriumPcd,probe_mesh,mesh_box_bottom])
                exit()


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
                #print(forces)
                pointForcePredictions = []
                for i in range(Ns): 
                    force = forces[i+Ns]-forces[i]
                    pointForcePredictions.append(force)
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

                #hsv's  hue: 0.25 = 0.0002 displacement 1 == 10 displacement
                #generate the entire displacement field
                #surfacePts are the equlibrium pts that provide 
                entireDisplacedPcd = []
                colorThreshold = 0.0005
                colorUpperBound = 0.012
                forceColorUpperBound = 0.004
                colorRange = colorUpperBound - colorThreshold
                greyColor = 0.38
                open3dPcd = PointCloud()
                forcePcd = PointCloud()
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
                    
                    (flag, counter) = inList(pt,surfacePts)
                    if not flag:
                        xyzForce.append(displacedDeformablePoint)
                        rgbForce.append([greyColor,greyColor,greyColor])
                    else:
                        pass
                    
                ##add the surface points for 
                for pt,aD,nD in zip(surfacePts,actualD,nominalD):
                    
                    estimatedNormal = vo.div(pt[6:9],vo.norm(pt[6:9]))
                    print(pt[0:3],vo.mul(estimatedNormal,aD))
                    displacedDeformablePoint = vo.sub(pt[0:3],vo.mul(estimatedNormal,nD-0.0003))
                    xyzForce.append(displacedDeformablePoint)
                    color = colorsys.hsv_to_rgb(aD/forceColorUpperBound*0.75+0.25,1,1)
                    color = [color[0],color[1],color[2]]
                    rgbForce.append(color)
                open3dPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
                open3dPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
                forcePcd.points = Vector3dVector(np.asarray(xyzForce,dtype=np.float32))
                forcePcd.colors = Vector3dVector(np.asarray(rgbForce,dtype=np.float32))
                estimate_normals(open3dPcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
                estimate_normals(forcePcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
                #open3dPcd,ind=statistical_outlier_removal(open3dPcd,nb_neighbors=10,std_ratio=2) 
                #draw_geometries([open3dPcd,mesh_box_bottom])
                draw_geometries([forcePcd,mesh_box_bottom])
                #draw_geometries([equilibriumPcd])
                print('maxDisp',maxDisp)
                print('maxActualD',max(actualD))

            else:
                predictedForces.append(0)
                predictedTorques.append(0)
            #print originalNominalD
            #print nominalD,actualD
            #timeSpentQueryingDisplacement = time.time()- startTime2
            #print('Time spent querying a displacement',timeSpentQueryingDisplacement)
            #if NofSurfacePts > 0:
            #    print('Ratio:',timeSpentQueryingModel/timeSpentQueryingDisplacement)
        #print("actual displacemetn",actualD)
        #print(rigidPtsInContact)
        #plt.plot(rigidPtsInContact[:][1],rigidPtsInContact[:][2])
        #plt.show()
        predictedForcesAll.append(predictedForces)
        predictedTorquesAll.append(predictedTorques)
    return predictedForcesAll,predictedTorquesAll

exp_N = 1
param = params[exp_N-1]
exp_N = str(exp_N)
startTime = time.time()


###################Load data for line probe################################################# 
exp_path='../data_final/exp_' + exp_N + '/'
exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
#load line geometry data 
lineStarts,lineEnds,lineNormals,lineTorqueAxes = cal_line_data(exp_path)
#load line force torque data 
X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='fntn',logfile=None)
#load visual model
pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
points = np.array(pcd)
##calculate the range
max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , 
                (np.max(points[:,1])-np.min(points[:,1])) , 
                (np.max(points[:,2])-np.min(points[:,2])) ])
p_min = []
p_max = []
for i in range(3):
    p_min.append(np.min(points[:,i]))
    p_max.append(np.max(points[:,i]))

##The shoe have 2 layers... and the pcd needs to be modified
if exp_N == '4':
    pcd_cut = []
    for ele in pcd:
        if ele[1]-p_min[1] < 0.08:
            pass
        elif (ele[1]-p_min[1] < 0.1) and (ele[2]-p_min[2] < 0.03):
            pass
        elif (ele[1]-p_min[1] < 0.1) and (ele[2]-p_min[2] < 0.08) and (ele[0]-p_min[0] > 0.03):
            pass
        elif (ele[1]-p_min[1] < 0.14) and (ele[2]-p_min[2] < 0.03) and (ele[0]-p_min[0] <0.02):
            pass
        elif (ele[2]-p_min[2] < 0.02) and (ele[0]-p_min[0] <0.03):
            pass
        else:
            pcd_cut.append(deepcopy(ele))


##load point model
print('data loaded in',time.time()-startTime, 'seconds')
iterator = '10' #we use model trained on 10 locations
#model_path = '../../Kai/data_final_new/exp_' + exp_N +'_debiased/models/model_pt'+ iterator + '_id' + str(int(id))+'.pkl'
model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt'+ iterator + '.pkl'
model = load_model(model_path)
offset = p_min + [max_range]

##load the stl model
probe = read_triangle_mesh("Contact_Probe_linear.ply")
probe.compute_vertex_normals()
probe.paint_uniform_color([1,0,0])
#draw_geometries([probe])


dilution = 60 #dilute the ground truth force-displacement datapoints
discretization = 0.003 # in meters
#### generating testing and predicting data...
ypredForce = []
ypredTorque = []
ytestForce = []
ytestTorque = []
startTime = time.time()
#for i in range(len(lineStarts)):
#print 
#for i in np.arange(130,160,1):
for i in [110]:
    print(i)
    if exp_N == '2' and i==285: ## pcd have holes and we end up having duplicated points
        pass#print('point 361 skipped...')
    elif exp_N == '3' and i==1:
        pass
    elif exp_N == '5' and i==249:
        pass
    else:
        #load ground truth and predict 
        num_iter = [i]
        tmp1 = X[i][::dilution]
        tmp2 = Y[i][::dilution]
        #queryDList = [-0.008,-0.002,0,0.003,0.005,0.007,0.009, 0.012] 
        queryDList = [-0.008,-0.002,0,0.003,0.005] 
        #print(tmp1[:,7])
        if exp_N == '4':
            predictedForces,predictedTorques = predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd_cut,param,discretization,num_iter,queryDList,model,offset,p_min,p_max)           
        else:
            predictedForces,predictedTorques = predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset,p_min,p_max)
        
        print(predictedForces)