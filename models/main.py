from contact_force_solver_line import *
from contact_force_solver_sphere import *
#load point model

startTime = time.time()
'''
###### for line probe 
#exp_path='../../data/exp_3_debaised/'
exp_path='../../data/exp_1/'
exp_path_2 = '../../data/exp_1_debiased/'
#load line geometry data 
lineStarts,lineEnds,lineNormals,lineTorqueAxes = cal_line_data(exp_path)
#load line force torque data 
X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='ft',logfile=None)
#load visual model
pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
#pcd = np.array(pcd)

param = 0.03 # in meters
discretization = 0.003 # in meters
#Iterate through different probe locations
num_iter = [23]
queryDList = [0.012]
predictedForces = predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,X,Y,pcd,param,discretization,num_iter,queryDList)
print predictedForces



print 'Data Loaded in',time.time()-startTime,'seconds'
#exp_path='../../data/exp_3_debaised/'
exp_path='../../data/exp_1/'
exp_path_2 = '../../data/exp_1_debiased/'
#load visual model
pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
#pcd = np.array(pcd)
probedPcd = load_pcd(exp_path+'probePcd.txt',pcdtype='return_lines')
diameter = 0.03 #need to check this...
param = 0.03 # in meters
discretization = 0.003 # in meters
num_iter = [23]
queryDList = [0]
predict_circle(pcd,param,discretization,num_iter,queryDList)
'''

creat_sphere(0.1,[0,0,0],100)