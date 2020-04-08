import contact_force_solver_line as solver_line
import contact_force_solver_sphere as solver_sphere
import contact_force_solver_circle as solver_circle
from data_loader import *
import time
from sklearn.metrics import mean_squared_error
import math
from copy import deepcopy
from scipy import signal
import matplotlib.pyplot as plt
def mean_squared_error_vector(A,B):
    total = 0.0
    for a,b in zip(A,B):
        #print(a,b)
        error = vo.norm(vo.sub(a,b))
        total = total + error*error
    return total/float(len(A))


Wn = 0.02
[b,a] = signal.butter(5,Wn,'low')




#load point model
#hand-picked param for 5 exps are:
#params = [0.03,0.15,0.02,0.01,0.008]
#new params with extended point data: [0.03,0.15,0.02,0.2,0.06]
params = [0.03,0.15,0.05,0.2,0.06]
forceRMSE = []
torqueRMSE = []
#exp_N = '1'

################################
#   code for line probe error  #
#                              #
################################
'''
#iterate through 5 different objects
for exp_N in [4,5]:
    #for param in [0.4]:
    #for param in [100]:
    if False:
        pass
    else:
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
        print('data loaded in',time.time()-startTime, 'seconds')
        iterator = '10' #we use model trained on 10 locations
        #model_path = '../../Kai/data_final_new/exp_' + exp_N +'_debiased/models/model_pt'+ iterator + '_id' + str(int(id))+'.pkl'
        #model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt'+ iterator + '_old'+'.pkl'
        model_path = '../data_final/exp_' + exp_N +'_debiased/models/model_pt'+ iterator +'.pkl'
        model = load_model(model_path)
        offset = p_min + [max_range]

        dilution = 100 #dilute the ground truth force-displacement datapoints
        
        ###### change this....
        discretization = 0.006 # in meters


        #### generating testing and predicting data...
        ypredForce = []
        ypredTorque = []
        ytestForce = []
        ytestTorque = []
        startTime = time.time()
        filterFlag = 0

        #iterate through pokes
        for i in range(len(lineStarts)):
        #for i in np.arange(150,180,1):
        #for i in np.arange(1,30,1):
        #for i in [28]:
        #for i in [110]:
            print(i)
            if exp_N == '2' and i==285: ## pcd have holes and we end up having duplicated points
                pass#print('point 361 skipped...')
            elif exp_N == '3' and i==1:
                pass
            elif exp_N == '3' and i==17:
                pass
            elif exp_N == '5' and i==249:
                pass
            elif exp_N == '4' and i ==3:
                pass
            else:
                #load ground truth and predict 
                num_iter = [i]
                #print(Y[i][:,0].tolist())
                
                #print(filteredForces)
                #plt.plot(filteredForces)
                #plt.plot(Y[i][:,0].tolist())
                #plt.show()
                #exit()
                tmp1 = X[i][::dilution]
                if filterFlag:
                    filteredForces = signal.lfilter(b,a,Y[i][:,0].tolist())
                    filteredTorques = signal.lfilter(b,a,Y[i][:,1].tolist())
                else:
                    tmp2 = Y[i][::dilution]
                queryDList = tmp1[:,7]
                #print(queryDList)
                #queryDList = np.linspace(-0.01,0.003056,20)
                if exp_N == '4':
                    predictedForces,predictedTorques = solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd_cut,param,discretization,num_iter,queryDList,model,offset)           
                else:
                    predictedForces,predictedTorques = solver_line.predict_line(lineStarts,lineEnds,lineNormals,lineTorqueAxes,pcd,param,discretization,num_iter,queryDList,model,offset)
                
                if i == 0:
                    if filterFlag:
                        ytestForce = np.array(filteredForces)[::dilution]
                        ytestTorque = np.array(filteredTorques)[::dilution]
                    else:
                        ytestForce = tmp2[:,0]
                        ytestTorque = tmp2[:,1]
                else:
                    if filterFlag:
                    
                        ytestForce = np.hstack((ytestForce,np.array(filteredForces)[::dilution]))
                        ytestTorque = np.hstack((ytestTorque,np.array(filteredTorques)[::dilution]))
                    else:
                        ytestForce = np.hstack((ytestForce,tmp2[:,0]))
                        ytestTorque = np.hstack((ytestTorque,tmp2[:,1]))
                
                #print(predictedForces[0])
                #print(queryDList,tmp2[:,0])
                ypredForce = ypredForce + predictedForces[0]
                ypredTorque = ypredTorque + predictedTorques[0]

                
                #plt.plot(list(queryDList),predictedForces[0],'r',list(queryDList),list(tmp2[:,0]),'g')#,queryDList,predictedTorques)#,'b',,queryDList,tmp2[:,1],'k')
                #plt.plot(list(queryDList),predictedTorques[0],'r',list(queryDList),list(tmp2[:,1]),'g')
                #plt.show()

                #print('ytestTorque',ytestTorque.shape)
                #print('ypredForce',len(ypredForce))
                #print('ypredTorque',len(ypredTorque))
        #print('predictedForce',ypredForce)
        forceRMSE.append(math.sqrt(mean_squared_error(ytestForce,ypredForce)))
        torqueRMSE.append(math.sqrt(mean_squared_error(ytestTorque,ypredTorque)))
        #print(math.sqrt(mean_squared_error(ytestForce,ypredForce)),math.sqrt(mean_squared_error(ytestTorque,ypredTorque)))
        print('Predictions finished in',time.time()-startTime, 'seconds')   
        print('RMSE for Force:',forceRMSE)
        print('RMSE for Torque:',torqueRMSE)


'''
################################
# code for cylinder probe error#
#                              #
################################
######################Load Data for Cylinder Probe ##################
forceRMSE = []
torqueRMSE = []
for exp_N in [2]:
    param = params[exp_N-1] # in meters
    exp_N = str(exp_N)
    exp_path='../data_final/exp_' + exp_N + '/'
    exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
    #load visual model  #### there could be some error here........ probe_type line??
    pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
    X,Y = load_data(exp_path, probe_type='circle', Xtype='loc_color_cur',ytype='fnt',logfile=None)
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
    model_path = '../data_final/exp_' + exp_N +'_debiased/models_expand/model_pt10.pkl'
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
    #for i in range(20):
        print('point location:',i)
        if i == 70 and exp_N == '4':
            pass
        elif exp_N == '3' and i==17: ##this one and the one below probe already in contact initially
                pass
        elif exp_N == '4' and i ==3:
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
