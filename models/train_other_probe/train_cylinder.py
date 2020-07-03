from data_loader import *
import autosklearn.regression
import sklearn
import pickle
import matplotlib.pyplot as plt
import numpy as np
from train_config import train_indexes

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

def cal_range(r2,mse,mae,num_train_sizes,r2_limit):

    r2 = np.array(r2)
    mse = np.array(mse)
    mae = np.array(mae)

    r2 = np.reshape(r2,(num_train_sizes,10))
    mse = np.reshape(mse,(num_train_sizes,10))
    mae = np.reshape(mae,(num_train_sizes,10))

    r2_mean = []
    mse_mean = []
    mae_mean = []

    r2_std = []
    mse_std = []
    mae_std = []

    for i in range(num_train_sizes):
        fix_index = (r2[i]>r2_limit)

        temp_r2 = r2[i][fix_index]
        #print(temp_r2)
        temp_r2_mean = np.mean(temp_r2)
        temp_r2_std = np.std(temp_r2)

        temp_mse = mse[i][fix_index]**0.5
        temp_mse_mean = np.mean(temp_mse)
        temp_mse_std = np.std(temp_mse)

        temp_mae = mae[i][fix_index]
        temp_mae_mean = np.mean(temp_mae)
        temp_mae_std = np.std(temp_mae)

        r2_mean.append(temp_r2_mean)
        r2_std.append(temp_r2_std)
        mse_mean.append(temp_mse_mean)
        mse_std.append(temp_mse_std)
        mae_mean.append(temp_mae_mean)
        mae_std.append(temp_mae_std)

        #print(temp_r2_mean,temp_mse_mean,temp_mae_mean)
    #exit()

    r2_mean = np.array(r2_mean)
    mse_mean = np.array(mse_mean)
    mae_mean = np.array(mae_mean)

    r2_std = np.array(r2_std)
    mse_std = np.array(mse_std)
    mae_std = np.array(mae_std)
    
    return r2_mean,mse_mean,mae_mean,r2_std,mse_std,mae_std

def main():
    #load data
    model_index = 0    
    X,y = load_data('./', probe_type='ellipse', Xtype='loc',ytype='t',logfile=None)
    
    r2_mean = []
    r2_std=[]
    rmse_mean=[]
    rmse_std = []
    #loop
    for i in range(len(train_indexes)):
        current_indexes = train_indexes[i]
        #analysis
        rmses = []

        for j in range(len(current_indexes)):
            if model_index < 0:
            #if current_indexes[j] != [88]:
                model_index=model_index+1
                print(model_index)
            else:
                logfile = open('model_log_singlemodel2.txt','a+')

                train_index = current_indexes[j]
                #multiply 3:
                #for ii in range(len(train_index)):
                #    train_index[ii] = train_index[ii]*3
                test_index = [x for x in list(range(len(X))) if x not in train_index]
                X_train, X_test, y_train, y_test= index_train_test_split(X, y, train_index,test_index)
                print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape))
                print(y_train.shape)
                train_size = X_train.shape[0]/(X_train.shape[0]+X_test.shape[0])
                print('train size', train_size)
                
                if train_size>0.25:
                    X_train = X_train[::4]
                    X_test = X_test[::4]
                    y_train = y_train[::4]
                    y_test = y_test[::4]
                
                automl_models = []
                for automl_i in range(3):
                    automl = autosklearn.regression.AutoSklearnRegressor(
                            time_left_for_this_task=30,#60*time_limit,
                            per_run_time_limit=2,#30*time_limit,
                            include_estimators = ['extra_trees'],
                            )
                    automl_models.append(automl)
                
                for automl_i in range(3):
                    automl_models[i].fit(X_train, y_train[:,i])
                    print(automl_models[i].show_models())
                
                #uniform downsample
                
                #print('[*]dataset downsample! Train: %s, Test: %s'%(X_train.shape,X_test.shape))
                predictions=[]
                for automl_i in range(3):
                    tmp_predictions = automl_models[i].predict(X_test)
                    predictions.append(tmp_predictions)
                
                predictions = np.vstack((predictions[0],predictions[1],predictions[2]))
                predictions = predictions.T
                rmse = np.mean(np.sum((predictions - y_test)**2,axis=1)**0.5)
                
                print('[*]rmse:',rmse)
                
                rmses.append(rmse)
                '''
                s = pickle.dumps(automl)
                with open('./ellipse_models/model_pt'+str(len(train_index))+'_id'+str(model_index)+'_t.pkl', 'wb') as f:
                    f.write(s)
                print('[*]save model!')
                '''
                model_index = model_index + 1
                
                #print('\n\n\n\n',file=logfile)
                logfile.close()
        
        rmses = np.array(rmses)
        temp_rmses_mean = np.mean(rmses)
        temp_rmses_std = np.std(rmses)

        rmse_mean.append(temp_rmses_mean)
        rmse_std.append(temp_rmses_std)
        print('\n\n\n','[*]calculate:',rmse_mean,rmse_std,'\n\n\n')
        
        exit()
    print('all done')
#load data

if __name__ == "__main__":
    main()


