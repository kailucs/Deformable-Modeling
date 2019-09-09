import contact_force_solver_line as solver_line
import contact_force_solver_sphere as solver_sphere
import contact_force_solver_circle as solver_circle
from data_loader import *
import time
#load point model

exp_N = '1'

startTime = time.time()

'''
###################Load data for line probe################################################# 
exp_path='../data_final/exp_' + exp_N + '/'
exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'

#load line geometry data 
lineStarts,lineEnds,lineNormals,lineTorqueAxes = cal_line_data(exp_path)
#load line force torque data 
X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='ft',logfile=None)
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
model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt10.pkl'
model = load_model(model_path)
offset = p_min + [max_range]
#print(offset)
print('data loaded in',time.time()-startTime, 'seconds')



param = 0.03 # in meters
discretization = 0.003 # in meters
num_iter = [23]
queryDList = [-0.005,-0.003,-0.001,0.001,0.003,0.005,0.007,0.009,0.011]
startTime = time.time()
predictedForces,predictedTorques = solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset)
print('forces:',predictedForces)
print('torques:',predictedTorques)
print('Time spend predicting force:',time.time()-startTime)

'''
'''
######################Load Data for Cylinder Probe ##################
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


diameter = 0.03 #need to check this...
param = 0.03 # in meters
discretization = 0.003 # in meters
num_iter = [23]
queryDList = [-0.005,-0.003,-0.001,0.001,0.003,0.005,0.007,0.009,0.011]
rigidPointsLocal = solver_circle.create_circle(discretization)
print('There is a total of',len(rigidPointsLocal),'rigid surface pts')
predictedForces,predictedTorques = solver_circle.predict_circle(pcd,probedPcd,rigidPointsLocal,param,discretization,num_iter,queryDList,model,offset)
print('forces:',predictedForces)
print('torques:',predictedTorques)

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