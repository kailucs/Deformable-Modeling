import contact_force_solver_line as solver_line
import contact_force_solver_sphere as solver_sphere
import contact_force_solver_circle as solver_circle
from data_loader import *
import time
from sklearn.metrics import mean_squared_error
import math
#load point model

exp_N = '1'

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
for i in range(3):
    p_min.append(np.min(points[:,i]))
#pcd = np.array(pcd)

##load point model
print('data loaded in',time.time()-startTime, 'seconds')
forceRMSE = []
torqueRMSE = []
#for iterator in ['2','3','4','5','6','7','8','9','10']:
iterator = '2'
for id in np.arange(162,193,1):
    model_path = '../../Kai/data_final_new/exp_' + exp_N +'_debiased/models/model_pt'+ iterator + '_id' + str(int(id))+'.pkl'
    model = load_model(model_path)
    offset = p_min + [max_range]

    #### generating testing and predicting data...
    ypredForce = []
    ypredTorque = []
    ytestForce = []
    ytestTorque = []

    dilution = 60
    param = 0.03 # in meters
    discretization = 0.003 # in meters
    startTime = time.time()
    for i in range(len(lineStarts)):
        if i == 361:
            pass#print('point 361 skipped...')
        else:
            #print('point location:',i)
            num_iter = [i]
            tmp1 = X[i][::dilution]
            tmp2 = Y[i][::dilution]
            #print (len(tmp1))
            queryDList = tmp1[:,7]
            predictedForces,predictedTorques = solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset)
            
            if i == 0:
                ytestForce = tmp2[:,0]
                ytestTorque = tmp2[:,1]
            else:
                ytestForce = np.hstack((ytestForce,tmp2[:,0]))
                ytestTorque = np.hstack((ytestTorque,tmp2[:,1]))
            ypredForce = ypredForce + predictedForces[0]
            ypredTorque = ypredTorque + predictedTorques[0]
            #print('ytestTorque',ytestTorque.shape)
            #print('ypredForce',len(ypredForce))
            #print('ypredTorque',len(ypredTorque))
    forceRMSE.append(math.sqrt(mean_squared_error(ytestForce,ypredForce)))
    torqueRMSE.append(math.sqrt(mean_squared_error(ytestTorque,ypredTorque)))
    print(math.sqrt(mean_squared_error(ytestForce,ypredForce)),math.sqrt(mean_squared_error(ytestTorque,ypredTorque)))
    print('Num of Points: ' + iterator)
    print('Predictions finished in',time.time()-startTime, 'seconds')   
    print('RMSE for Force:',forceRMSE)
    print('RMSE for Torque:',torqueRMSE)







'''

######################Load Data for Cylinder Probe ##################
forceRMSE = []
#torqueRMSE = []
for iterator in ['2','3','4','5','6','7','8','9','10']:
    exp_path='../data_final/exp_' + exp_N + '/'
    exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
    #load visual model
    pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
    X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='fn',logfile=None)
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

    ##load point model
    model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt10.pkl'
    model = load_model(model_path)
    offset = p_min + [max_range]

    print('Data Loaded in',time.time()-startTime,'seconds')

    #### generating testing and predicting data...
    ypredForce = []
    #ypredTorque = []
    ytestForce = []
    #ytestTorque = []

    dilution = 30
    param = 0.03 # in meters
    discretization = 0.003 # in meters
    rigidPointsLocal = solver_circle.create_circle(discretization)
    startTime = time.time()
    for i in range(len(lineStarts)):
        print('point location:'i)
        num_iter = [i]
        tmp1 = X[i][::dilution]
        tmp2 = Y[i][::dilution]
        print (len(tmp1))
        queryDList = tmp1[:,7]
        predictedForces,predictedTorques = solver_circle.predict_circle(pcd,probedPcd,rigidPointsLocal,param,discretization,num_iter,queryDList,model,offset)

        if i == 0:
            ytestForce = tmp2[:,0]
            #ytestTorque = tmp2[:,1]
        else:
            ytestForce = np.hstack((ytestForce,tmp2[:,0]))
            #ytestTorque = np.hstack((ytestTorque,tmp2[:,1]))
        ypredForce = ypredForce + predictedForces[0]
        #ypredTorque = ypredTorque + predictedTorques[0]
        #print('ytestForce',ytestForce.shape)
        #print('ytestTorque',ytestTorque.shape)
        #print('ypredForce',len(ypredForce))
        #print('ypredTorque',len(ypredTorque))
    forceRMSE.append(math.sqrt(mean_squared_error(ytestForce,ypredForce)))
    print('Predictions finished in',time.time()-startTime, 'seconds')    
    print('RMSE for Force:',math.sqrt(mean_squared_error(ytestForce,ypredForce)))
    #print('RMSE for Torque:',math.sqrt(mean_squared_error(ytestTorque,ypredTorque)))
print('RMSE for Force:',forceRMSE)

'''

'''
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
for i in range(3):
    p_min.append(np.min(points[:,i]))

##load point model
model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt10.pkl'
model = load_model(model_path)
offset = p_min + [max_range]
print('Data Loaded in',time.time()-startTime,'seconds')

diameter = 0.1 #need to check this...
param = 0.03 # in meters
discretization = 0.003 # in meters
num_iter = [23]
queryDList = [0.001]

rigidSurfacePtsAll = solver_sphere.create_sphere(diameter/2.0,[0,0,0],400)
deformableCenter = [-0.5,0.1,0.01]
rigidCenter = [-0.5,0.1,1.01]
for i in [-0.002,-0.001,0,0.001,0.002,0.003,0.008]:
    rigidCenter2 = vo.sub(rigidCenter,[0,0,i])
    deformableCenter2 = vo.sub(deformableCenter,[0,0,i])
    force =solver_sphere.predict_sphere(pcd,probedPcd,rigidSurfacePtsAll,param,discretization,num_iter,queryDList,model,offset,deformableCenter2,rigidCenter2,diameter)
    print('force:',force)
'''