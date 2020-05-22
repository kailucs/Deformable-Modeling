import contact_force_solver_line as solver_line
#import contact_force_solver_sphere as solver_sphere
import contact_force_solver_circle as solver_circle
from data_loader import *
import time
from sklearn.metrics import mean_squared_error
import math
from copy import deepcopy
from scipy import signal
import matplotlib.pyplot as plt
import open3d
import numpy as np
from klampt import vis, PointCloud
from klampt import WorldModel,Geometry3D
from klampt.io import loader
def mean_squared_error_vector(A,B):
    total = 0.0
    for a,b in zip(A,B):
        #print(a,b)
        error = vo.norm(vo.sub(a,b))
        total = total + error*error
    return total/float(len(A))

def rgb_int2tuple(rgbint):
    return ((rgbint // 256 // 256 % 256)/255.0, (rgbint // 256 % 256)/255.0, (rgbint % 256)/255.0)

#c constant for the 5 objects:
params = [0.03,0.15,0.02,0.04,0.012]
exp_N = '1'


################################
#   code for line probe error  #
#                              #
################################

#iterate through 5 different objects
for exp_N in [5]:#,2,4,5]:#,2,3,4,5]:

    #initiate the world
    w = WorldModel()

    #add a floor
    # floor =  Geometry3D()
    # floor.loadFile("cube.off")
    # floor_length = 0.2
    # floor_width = 0.2
    # floor.transform([floor_length,0,0,0,floor_width,0,0,0,0.01],[-floor_length/2.0,-floor_width/2.0,-0.01])
    # floor_terrain = w.makeTerrain("floor")
    # floor_terrain.geometry().set(floor)
    # floor_terrain.appearance().setColor(0.4,0.3,0.2,1.0)

    #add the probe
    item_1_geom = Geometry3D()
    item_1_geom.loadFile('Contact_Probe_linear2.STL') 
    #item_1_geom.transform(T_g[0],T_g[1])
    item_1 = w.makeRigidObject('probe')
    item_1.geometry().set(item_1_geom)
    item_1.appearance().setColor(1,0,0,0.2)
    


    ###################Load data for line probe################################################# 
    exp_path='../data_final/exp_' + str(exp_N) + '/'
    exp_path_2 = '../data_final/exp_' + str(exp_N) + '_debiased/'
    #load line geometry data 
    lineStarts,lineEnds,lineNormals,lineTorqueAxes = cal_line_data(exp_path)
    #load visual model
    pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
    points = np.array(pcd)
    
    ##calculate the range
    max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , 
                    (np.max(points[:,1])-np.min(points[:,1])) , 
                    (np.max(points[:,2])-np.min(points[:,2])) ])
    p_min = []
    for i in range(3):
        p_min.append(np.min(points[:,i]))

    ##The shoe have 2 layers... and the pcd needs to be modified
    if exp_N == 4:
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

    ### rigid object discretization
    discretization = 0.002 # in meters

    #add the point cloud#open3dPcd = open3d.geometry.PointCloud()
    open3dPcd = open3d.geometry.PointCloud()
    xyz = []
    rgb = []
    if exp_N != 4:
        for ele in pcd:
            xyz.append(ele[0:3])
            rgb.append(ele[3:6])
        open3dPcd.points = open3d.utility.Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd.colors = open3d.utility.Vector3dVector(np.asarray(rgb,dtype=np.float32))
        open3d.io.write_point_cloud("testPcd.pcd", open3dPcd)   
    else:
        for ele in pcd_cut:
            xyz.append(ele[0:3])
            rgb.append(ele[3:6])
        open3dPcd.points = open3d.utility.Vector3dVector(np.asarray(xyz,dtype=np.float32))
        open3dPcd.colors = open3d.utility.Vector3dVector(np.asarray(rgb,dtype=np.float32))
        open3d.io.write_point_cloud("testPcd.pcd", open3dPcd)   

    pcdK = loader.loadGeometry3D("testPcd.pcd")
    klampt_pcd = pcdK.getPointCloud()
    properties = np.array(klampt_pcd.properties).reshape((klampt_pcd.numPoints(),klampt_pcd.numProperties()))
    pcd_new = PointCloud()
    rs =[]
    gs = []
    bs =[]
    for a in properties:
        position = a[0:3]
        pcd_new.addPoint(position)
        rgb = a[3] 
        (r,g,b) = rgb_int2tuple(rgb)
        rs.append(r)
        gs.append(g)
        bs.append(b)

    pcd_new.addProperty('r',rs)
    pcd_new.addProperty('g',gs)
    pcd_new.addProperty('b',bs)
    pcd_new_geom = Geometry3D()
    pcd_new_geom.setPointCloud(pcd_new)

    pcd_object = w.makeRigidObject('pcd')
    pcd_object.geometry().set(pcd_new_geom)
    vis.add("world",w)



    param = params[exp_N-1]
    exp_N = str(exp_N)
    startTime = time.time()

    #iterate through pokes
    for i in [60]:
        print(i)
        num_iter = [i]
        #in some of these cases, the pcd my have holes, and we end up having duplicated points when doing
        #neighbor searches
        if exp_N == '2' and i==285: ## pcd have holes and we end up having duplicated points
            pass#print('point 361 skipped...')
        elif exp_N == '3' and i==1:
            pass
        elif exp_N == '5' and i==249:
            pass
        else:
            #a list of displacement
            queryDList = np.linspace(0,0.0085, num=30)
            if exp_N == '4':
                solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd_cut,param,discretization,num_iter,queryDList,vis,w)           
            else:
                solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,vis,w)
            


'''
################################
# code for cylinder probe error#
#                              #
################################
######################Load Data for Cylinder Probe ##################
forceRMSE = []
torqueRMSE = []
for exp_N in [1,2,3,4,5]:
    param = params[exp_N-1] # in meters
    exp_N = str(exp_N)
    exp_path='../data_final/exp_' + exp_N + '/'
    exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
    #load visual model
    pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
    X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='fnt',logfile=None)
    #pcd = np.array(pcd)
    probedPcd = load_pcd(exp_path+'probePcd.txt',pcdtype='return_lines')

    points = np.array(pcd)
    ##calculate the range
    max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , 
                    (np.max(points[:,1])-np.min(points[:,1])) , 
                    (np.max(points[:,2])-np.min(points[:,2])) ])
    p_min = []
    for i in range(3):
        p_min.append(np.min(points[:,i]))

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
    model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt10.pkl'
    model = load_model(model_path)
    offset = p_min + [max_range]

    #print('Data Loaded in',time.time()-startTime,'seconds')

    #### generating testing and predicting data...
    ypredForce = []
    ypredTorque = []
    ytestForce = []
    ytestTorque = []

    dilution = 30
    
    discretization = 0.003 # in meters
    rigidPointsLocal = solver_circle.create_circle(discretization)
    startTime = time.time()
    for i in range(len(probedPcd)):
    #for i in [0,1]:
        print('point location:',i)
        if i == 70 and exp_N == '4':
            pass
        else:
            num_iter = [i]
            tmp1 = X[i][::dilution]
            tmp2 = Y[i][::dilution]
            queryDList = tmp1[:,7]
            if exp_N == '4':
                predictedForces,predictedTorques = solver_circle.predict_circle(pcd_cut,probedPcd,rigidPointsLocal,param,discretization,num_iter,queryDList,model,offset)
            else:
                predictedForces,predictedTorques = solver_circle.predict_circle(pcd,probedPcd,rigidPointsLocal,param,discretization,num_iter,queryDList,model,offset)

            if i == 0:
                ytestForce = tmp2[:,0]
                ytestTorque = tmp2[:,1:4]
            else:
                #print(tmp2[:,1:4].shape)
                ytestForce = np.hstack((ytestForce,tmp2[:,0]))
                ytestTorque = np.vstack((ytestTorque,tmp2[:,1:4]))
            #print(ytestTorque.shape)
            #exit()
            #print(predictedForces[0],predictedTorques[0])
            ypredForce = ypredForce + predictedForces[0]
            ypredTorque = ypredTorque + predictedTorques[0]
            #print(ytestForce,ytestTorque,ypredForce,ypredTorque)

    forceRMSE.append(math.sqrt(mean_squared_error(ytestForce,ypredForce)))
    torqueRMSE.append(math.sqrt(mean_squared_error_vector(ytestTorque,ypredTorque)))
    print('Predictions finished in',time.time()-startTime, 'seconds')    
    print('RMSE for Force:',forceRMSE)
    print('RMSE for Torque:',torqueRMSE)
'''