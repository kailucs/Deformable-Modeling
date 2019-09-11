from data_loader import *
import time
from sklearn.metrics import mean_squared_error
import math
from scipy import spatial
from copy import deepcopy
from klampt.math import se3,so3 
import matplotlib.pyplot as plt
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

def predict_sphere(pcd,probedPcd,rigidSurfacePtsAll,param,model,offset,deformableCenter,rigidCenter,diameter,OPEN3DVIS = False):
    DEBUGPROJECTEDPTS = False
    DEBUGDISPLACEDPTS = False
    #OPEN3DVIS = False
    #create a pcd in open3D
    #if OPEN3DVIS:
    if OPEN3DVIS:
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
        if vo.dot(tmp,approachVector) > 0.1:
            projectedPt = vo.sub(pt,vo.mul(approachVector,vo.dot(vo.sub(pt,rigidCenter),approachVector))) ##world frame
            projectedPt2D = vo.sub(projectedPt,rigidCenter) #world frame
            projectedPt2DinLocal = [vo.dot(projectedPt2D,localXinW),vo.dot(projectedPt2D,localYinW)]
            rigidPointsLocal.append(projectedPt2DinLocal)
            rigidPointsinW.append(pt)

    #############Now Start predicting force#####################

    ######## visualize in open3D for debugging ########
    #if OPEN3DVIS:
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
            if d[0] < 0.003:
                surfacePt = [0]*10
                for j in range(NofN):
                    surfacePt = vo.add(surfacePt,projectedPcdinW[Idx[j]][0:10])
                surfacePt = vo.div(surfacePt,NofN)
                surfacePtsAll.append(surfacePt)        
    
        surfacePts = [] #These are the surface pts that will be displaced...
        nominalD = [] #Column Vector..
        rigidPtsInContact = []
        for i in range(len(rigidPointsinW)):
            rigidPt = rigidPointsinW[i]
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

        #originalNominalD = deepcopy(nominalD)
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
            #predictedTorques.append(totalTorque)
            #print("actual displacement:",actualD)
            #startTime = time.time()
            for i in range(len(surfacePts)):
                queryPt = surfacePts[i][0:3] + [actualD[i]]
                queryPt = np.array(queryPt,ndmin=2)
                #queryPt = normalize_points(queryPt,[offset[0:3]],offset[3])
                queryPt = normalize_points(queryPt,[[0,0,0]],offset[3])
                force = model.predict(queryPt)
                totalForce = totalForce + force[0]
                #torqueArm = vo.sub(rigidPtsInContact[i],torqueCenter)
                #normal = surfacePts[i][6:9]
                #torque = vo.cross(torqueArm,vo.mul(normal,force[0]))
                #totalTorque = vo.add(totalTorque,torque)  
            #timeSpentQueryingModel = time.time()- startTime
            #print('Time spent querying point model',timeSpentQueryingModel)
        
        return totalForce
    return 0


#MODE = 1 #run simulation
MODE = 2 #plot



startTime = time.time()
exp_N = '1'
################## Planning problem for sphere ######################
exp_path='../data_final/exp_' + exp_N + '/'
exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
#load visual model
pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
#pcd = np.array(pcd)
probedPcd = load_pcd(exp_path+'probePcd.txt',pcdtype='return_lines')
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

###shift the pcd..
pcd_shifted = []
for pt in pcd:
    pcd_shifted.append(vo.sub(pt[0:3],p_min)+pt[3:9])

probedPcd_shifted = []
for pt in probedPcd:
    probedPcd_shifted.append(vo.sub(pt[0:3],p_min)+pt[3:9])

##load point model
model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt10.pkl'
model = load_model(model_path)
offset = p_min + [max_range]
print('Data Loaded in',time.time()-startTime,'seconds')




#rigid object paramters
diameter = 0.05 

if MODE == 1:


    #simulation parameter
    param = 0.03 # in meters
    discretization = 0.003 # in meters
    rigidSurfacePtsAll = create_sphere(diameter/2.0,[0,0,0],200)

    # search x-y grid
    gridDiscretization = 0.005
    gridBoundX = [diameter/2.0 , p_max[0]-p_min[0] - diameter/2.0]
    gridBoundY = [diameter/2.0 , p_max[1]-p_min[1] - diameter/2.0]
    NX = round((gridBoundX[1] - gridBoundX[0])/gridDiscretization)
    NY = round((gridBoundY[1] - gridBoundY[0])/gridDiscretization)
    gridPtsX = np.linspace(gridBoundX[0],gridBoundX[1],int(NX))
    gridPtsY = np.linspace(gridBoundY[0],gridBoundY[1],int(NY))

    searchStartZ = p_max[2] - p_min[2] + diameter/2.0 + 0.001
    deformableCenter = [(gridBoundX[0]+gridBoundX[1])/2.0,(gridBoundY[0]+gridBoundY[1])/2.0,0]
    forceThreshold = 2 
    searchDiscretization = 0.001
    minZs = []

    for x in gridPtsX:
        for y in gridPtsY:
            forceMagnitude = 0
            z = searchStartZ
            while(forceMagnitude < forceThreshold):
                rigidCenter = [x,y,z]
                print(rigidCenter)
                if z >= diameter/2.0:
                    force = predict_sphere(pcd_shifted,probedPcd_shifted,rigidSurfacePtsAll,param,model,offset,deformableCenter,rigidCenter,diameter,True)
                    z = z - searchDiscretization
                    forceMagnitude = math.fabs(force)
                    if forceMagnitude > forceThreshold:
                        minZs.append(vo.add(rigidCenter,[0,0,searchDiscretization]))
                        break
                else:
                    minZs.append[x,y,diameter/2.0]
                    break    
                exit()

if MODE == 2:
    open3dPcd = PointCloud()
    xyz = []
    rgb = []
    for ele in pcd_shifted:
        xyz.append(ele[0:3])
        rgb.append(ele[3:6]) 
    open3dPcd.points = Vector3dVector(np.asarray(xyz,dtype=np.float32))
    open3dPcd.colors = Vector3dVector(np.asarray(rgb,dtype=np.float32))
    rigidCenter = [0.1,0.1,0.3]
    mesh_sphere = geometry.create_mesh_sphere(radius=diameter/2.0, resolution=30)
    sphere_transform = np.asarray(
                [[1, 0, 0,  rigidCenter[0]],
                [0, 1, 0,  rigidCenter[1]],
                [0, 0, 1, rigidCenter[2]],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_sphere.transform(sphere_transform)
    mesh_sphere.paint_uniform_color([235.0/255.0, 62.0/255.0, 14.0/255.0])

    #create box
    wallThickness = 0.005
    wallColor = [165.0/255.0, 111.0/255.0, 227.0/255.0]
    wallHeight = 0.15
    mesh_box_bottom = geometry.create_mesh_box(p_max[0]-p_min[0],p_max[1]-p_min[1],wallThickness) #x,y,z
    mesh_box_bottom.paint_uniform_color(wallColor)
    transform = np.asarray(
                [[1, 0, 0,  0],
                [0, 1, 0,  0],
                [0, 0, 1, -wallThickness],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_bottom.transform(transform)    

    mesh_box_left = geometry.create_mesh_box(p_max[0]-p_min[0]+2*wallThickness,wallThickness,wallHeight) #x,y,z
    mesh_box_left.paint_uniform_color(wallColor)
    transform = np.asarray(
                [[1, 0, 0,  -wallThickness],
                [0, 1, 0,  -wallThickness],
                [0, 0, 1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_left.transform(transform) 
    
    mesh_box_right = geometry.create_mesh_box(p_max[0]-p_min[0]+2*wallThickness,wallThickness,wallHeight) #x,y,z
    mesh_box_right.paint_uniform_color(wallColor)
    transform = np.asarray(
                [[1, 0, 0,  -wallThickness],
                [0, 1, 0,  p_max[1]-p_min[1]],
                [0, 0, 1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_right.transform(transform)

    mesh_box_front= geometry.create_mesh_box(wallThickness,p_max[1]-p_min[1],wallHeight) #x,y,z
    mesh_box_front.paint_uniform_color(wallColor)
    transform = np.asarray(
                [[1, 0, 0,  p_max[0]-p_min[0]],
                [0, 1, 0,  0],
                [0, 0, 1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_front.transform(transform)  

    mesh_box_back= geometry.create_mesh_box(wallThickness,p_max[1]-p_min[1],wallHeight) #x,y,z
    mesh_box_back.paint_uniform_color(wallColor)
    transform = np.asarray(
                [[1, 0, 0,  -wallThickness],
                [0, 1, 0,  0],
                [0, 0, 1, 0],
                [0.0, 0.0, 0.0, 1.0]])
    mesh_box_back.transform(transform) 

    open3d.visualization.draw_geometries([open3dPcd,mesh_sphere,mesh_box_bottom,mesh_box_left,mesh_box_right,mesh_box_front,mesh_box_back])