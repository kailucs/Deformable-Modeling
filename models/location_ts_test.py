import numpy as np
import autosklearn.regression
import sklearn.model_selection
import sklearn.datasets
import sklearn.metrics
import scipy
import pickle
import random

def load_data(point_path, force_path, probe_type='point', datatype='1'):
    
    points=[]
    colors=[]
    normals=[]
    curvatures=[]

    
    dataFile=open(point_path,'r')
    for line in dataFile:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        points.append(l2[0:3])
        colors.append(l2[3:6])
        normals.append(l2[6:9])
        curvatures.append(l2[9])
    dataFile.close()
    
    # normalize, note colors and normals is 0~1
    points = np.array(points)
    colors = np.array(colors)
    normals = np.array(normals)
    curvatures = np.array(curvatures)

    max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , (np.max(points[:,1])-np.min(points[:,1])) , (np.max(points[:,2])-np.min(points[:,2])) ])
    for i in range(3):
        points[:,i] = (points[:,i]-np.min(points[:,i]))/max_range

    num_point = len(points)
    print('[*]load %d points, and normalized'%num_point)

    '''
    X = np.array([[]])
    Y = np.array([[]])
    insert_i = 0
    '''
    X=[]
    Y=[]

    for i in range(num_point):
        force_path = './'+probe_type+'/force_'+str(i)+'.txt'
        force=[]
        force_normal=[]
        displacement=[]
        theta=[]

        dataFile=open(force_path,'r')
        for line in dataFile:
            line=line.rstrip()
            l=[num for num in line.split(' ')]
            l2=[float(num) for num in l]
            if probe_type == 'point':
                force.append(l2[0:3])
                force_normal.append(l2[3])
                displacement.append(l2[4])
            else:
                force.append(l2[0:3])
                force_normal.append(l2[3])
                displacement.append(l2[4])
                theta.append(l2[5:7])
        dataFile.close()

        # clean
        #TODO:

        # final
        if probe_type == 'point':
            num_dis = len(displacement)
            #print('---load %d displacement'%num_dis)
            displacement = np.resize(np.array(displacement),(num_dis,1)) 
            X_i = np.hstack((np.tile(points[i],(num_dis,1)), displacement))
            Y_i = np.array(force_normal,ndmin=2).T
            '''
            if insert_i == 0:
                X=X_i
                Y=Y_i
            else:
                X = np.vstack((X,X_i))
                Y = np.vstack((Y,Y_i))
            
            insert_i = insert_i + 1
            '''
            X.append(X_i)
            Y.append(Y_i)

    return X,Y

def my_train_test_split(X,y,num_point=1,train_size=0.8,select_method='random'):
    num_point = len(X)
    if select_method=='random':
        train_index = random.sample(range(num_point),int(train_size*num_point))
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='uniform':
        train_index = [int(i*(1.0/train_size)) for i in range(int(train_size*num_point))]
        test_index = [x for x in range(num_point) if x not in train_index]

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

def main():
    train_sizes = np.arange(0.1,1.0,0.1)
    model_index = 0
    logfile = open('model_log_mutimodel.txt','a+')
    
    #train_size = 0.75
    train_sizes = [0.8]
    model_index = 7
    for train_size in train_sizes:
    #for exp_index in range(1):
        X,y = load_data('./probePcd.txt','.') #note: is list  
        X_train, X_test, y_train, y_test = my_train_test_split(X, y, train_size=train_size,select_method='uniform')
        print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape),file=logfile)
        print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape))

        #uniform downsample
        X_train = X_train[::4]
        X_test = X_test[::4]
        y_train = y_train[::4]
        y_test = y_test[::4]
        
        print('[*]dataset downsample! Train: %s, Test: %s'%(X_train.shape,X_test.shape),file=logfile)
        print('[*]dataset downsample! Train: %s, Test: %s'%(X_train.shape,X_test.shape))
        
        automl = autosklearn.regression.AutoSklearnRegressor(
                time_left_for_this_task=600,
                per_run_time_limit=50,
                tmp_folder='/tmp/autosklearn_regression_dm',
                output_folder='/tmp/autosklearn_regression_dm_out',
                )
        
        automl.fit(X_train, y_train)
        y_hat = automl.predict(X_test)
        
        print(automl.show_models(),file=logfile)
        print(automl.show_models())
        predictions = automl.predict(X_test)
        print("[*]R2 score:", sklearn.metrics.r2_score(y_test, predictions),file=logfile)
        print("[*]R2 score:", sklearn.metrics.r2_score(y_test, predictions))

        s = pickle.dumps(automl)
        with open('model_mutimodels_'+str(model_index)+'.pkl', 'wb') as f:
            f.write(s)
        print('[*]save model!')
        model_index = model_index + 1
        
        print('\n\n\n\n',file=logfile)
    
    logfile.close()

if __name__ == "__main__":
    main()
