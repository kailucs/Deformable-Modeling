import numpy as np
import autosklearn.regression
import sklearn.model_selection
import sklearn.datasets
import sklearn.metrics
import scipy
import pickle
import random
from train_config import train_indexes

def load_data(exp_path, probe_type='point', Xtype='loc',logfile=None):
    
    points=[]
    colors=[]
    normals=[]
    curvatures=[]
    theta = []
    
    # load ori data
    dataFile=open(exp_path+'probePcd.txt','r')

    for line in dataFile:        
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]

        points.append(l2[0:3])
        colors.append(l2[3:6])
        normals.append(l2[6:9])
        curvatures.append(l2[9])
        
        if probe_type == 'line':
            theta.append(l2[10:13])

    dataFile.close()
    
    # normalize, note colors and normals is 0~1
    points = np.array(points)
    colors = np.array(colors)
    normals = np.array(normals)
    curvatures = np.array(curvatures)

    max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , 
                        (np.max(points[:,1])-np.min(points[:,1])) , 
                            (np.max(points[:,2])-np.min(points[:,2])) ])
    for i in range(3):
        points[:,i] = (points[:,i]-np.min(points[:,i]))/max_range

    num_point = len(points)
    
    print('[*]load %d points, and normalized'%num_point)
    if logfile != None:
            print('[*]load %d points, and normalized'%num_point,file=logfile)
    
    X=[]
    Y=[]

    for i in range(num_point):
        force_path = exp_path+probe_type+'/force_'+str(i)+'.txt'
        force=[]
        force_normal=[]
        displacement=[]
        theta=[]

        dataFile=open(force_path,'r')
        for line in dataFile:
            line=line.rstrip()
            l=[num for num in line.split(' ')]
            l2=[float(num) for num in l]
            force.append(l2[0:3])
            force_normal.append(l2[3])
            displacement.append(l2[4])
        dataFile.close()

        # final
        if probe_type == 'point':
            num_dis = len(displacement)
            #print('---load %d displacement'%num_dis)
            displacement = np.resize(np.array(displacement),(num_dis,1))
            if Xtype == 'loc':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 displacement))
            elif Xtype == 'loc_cur':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 np.tile(curvatures[i],(num_dis,1)), 
                                 displacement))
            elif Xtype == 'loc_color':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 np.tile(colors[i],(num_dis,1)), 
                                 displacement))

            Y_i = np.array(force_normal,ndmin=2).T
            X.append(X_i)
            Y.append(Y_i)

        elif probe_type == 'line':
            num_dis = len(displacement)
            #print('---load %d displacement'%num_dis)
            displacement = np.resize(np.array(displacement),(num_dis,1)) 
            X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                             np.tile(normals[i],(num_dis,1)), 
                             np.tile(theta[i],(num_dis,1)), 
                             displacement))
            Y_i = np.array(force_normal,ndmin=2).T
            X.append(X_i)
            Y.append(Y_i)

    return X,Y

def my_train_test_split(X,y,num_point=1,train_size=0.75,select_method='random',logfile=None):
    num_point = len(X)
    if select_method=='random':
        train_index = random.sample(range(num_point),int(train_size*num_point))
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='uniform':
        train_index = [int(i*(1.0/train_size)) for i in range(int(train_size*num_point))]
        test_index = [x for x in range(num_point) if x not in train_index]

    print('train points: %s \ntest points: %s'%(train_index,test_index))
    if logfile != None:
        print('train points: %s \ntest points: %s'%(train_index,test_index),file=logfile)

    flag = 0
    for i in train_index:
        if flag==0:
            X_train = X[i]
            y_train = y[i]
            flag = 1
        else:
            X_train = np.vstack((X_train,X[i]))
            y_train = np.vstack((y_train,y[i]))
    
    flag = 0
    for i in test_index:
        if flag==0:
            X_test = X[i]
            y_test = y[i]
            flag = 1
        else:
            X_test = np.vstack((X_test,X[i]))
            y_test = np.vstack((y_test,y[i]))

    return X_train, X_test, y_train, y_test

def index_train_test_split(X,y,train_index,test_index,logfile=None):
    print('train points: %s \ntest points: %s'%(train_index,test_index))
    if logfile != None:
        print('train points: %s \ntest points: %s'%(train_index,test_index),file=logfile)

    flag = 0
    for i in train_index:
        if flag==0:
            X_train = X[i]
            y_train = y[i]
            flag = 1
        else:
            X_train = np.vstack((X_train,X[i]))
            y_train = np.vstack((y_train,y[i]))
    
    flag = 0
    for i in test_index:
        if flag==0:
            X_test = X[i]
            y_test = y[i]
            flag = 1
        else:
            X_test = np.vstack((X_test,X[i]))
            y_test = np.vstack((y_test,y[i]))

    return X_train, X_test, y_train, y_test

def generate_index(num_point,train_size=0.75,select_method='random'):
    if select_method=='random':
        train_index = random.sample(range(num_point),int(train_size*num_point))
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='uniform':
        train_index = [int(i*(1.0/train_size)) for i in range(int(train_size*num_point))]
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='cv':
        num_pt = num_point
        split_set = []
        for num_pt_in_set in range(10,0,-1):
            point_indexes=list(range(num_pt))
            num_set = int(num_pt/num_pt_in_set)

            tmp_set2 = []
            iter = 0
            for i in range(num_set):
                tmp_set = []
                for ii in range(num_pt_in_set):
                    tmp_index = random.choice(point_indexes)
                    #print(tmp_index)
                    tmp_set.append(tmp_index)
                    point_indexes.remove(tmp_index)
                    print(iter)
                    iter = iter+1
                tmp_set2.append(tmp_set)

            split_set.append(tmp_set2)
            print(split_set)
            #TODO:
    return train_index,test_index

def run_training(exp_path,probe_type='point',Xtype='loc',train_size=0.75,select_method='random',ds_rate=1,
                time_left_for_this_task=360,per_run_time_limit=30,use_show_models=False,model_index=0):

    logfile = open('log/'+exp_path[-6:]+'model_log.txt','a+')
    
    print('[*]model '+model_index+' training process start',file=logfile)
    print('[*]model '+model_index+' training process start')

    X,y = load_data(exp_path,probe_type,Xtype,logfile) #note: is list 
             
    X_train, X_test, y_train, y_test = my_train_test_split(X, y,1,train_size,select_method,logfile)
    
    print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape),file=logfile)
    print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape))

    #uniform downsample
    X_train = X_train[::ds_rate]
    X_test = X_test[::ds_rate]
    y_train = y_train[::ds_rate]
    y_test = y_test[::ds_rate]

    print('[*]dataset downsample! Train: %s, Test: %s'%(X_train.shape,X_test.shape),file=logfile)
    print('[*]dataset downsample! Train: %s, Test: %s'%(X_train.shape,X_test.shape))

    automl = autosklearn.regression.AutoSklearnRegressor(
            time_left_for_this_task=time_left_for_this_task,
            per_run_time_limit=per_run_time_limit,
            #tmp_folder='/tmp/autosklearn_regression_dm',
            #output_folder='/tmp/autosklearn_regression_dm_out',
            )

    print('[*]Training start!')
    
    #core!
    automl.fit(X_train, y_train)

    if use_show_models == True:
        print(automl.show_models())

    predictions = automl.predict(X_test)
    
    r2_score = sklearn.metrics.r2_score(y_test, predictions)
    mse = sklearn.metrics.mean_squared_error(y_test, predictions)
    mae = sklearn.metrics.mean_absolute_error(y_test, predictions)
    
    print('[*]Training done!')

    print("[*]R2 score, mse, mae:\n", r2_score, mse, mae)
    print("[*]R2 score, mse, mae:\n", r2_score, mse, mae,file=logfile)
    
    s = pickle.dumps(automl)
    with open('model_mutimodels_'+str(model_index)+'.pkl', 'wb') as f:
        f.write(s)
    print('[*]save model!')
    print('\n\n\n\n',file=logfile)

    logfile.close()

if __name__ == "__main__":
    print(train_indexes)

            
