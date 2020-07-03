import autosklearn.regression
import sklearn
import numpy as np
from models.train_config import train_indexes_dense #TODO: del 'models.' if put back into data folder
import pickle
from models.data_loader import * #TODO: del 'models.' if put back into data folder

def main():
    # load config
    train_indexes = train_indexes_dense
    #load data
    model_index = 0
    print('index loaded')
    X,y = load_data('./', probe_type='point', Xtype='loc',ytype='fn',logfile=None)#note: is list
    # TODO: this file is originally in the data folder so use './'. Need to be fixed.

    r2_mean = []
    r2_std=[]
    rmse_mean=[]
    rmse_std = []
    print('data loaded')
    #loop
    for i in range(len(train_indexes)):
        current_indexes = train_indexes[i]

        #analysis
        r2_scores = []
        rmses = []

        for j in range(len(current_indexes)):
            if model_index < 0:
            #if current_indexes[j] != [88]:
                model_index=model_index+1
                print(model_index)
            else:
                logfile = open('model_log_AutoET.txt','a+')

                train_index = current_indexes[j]
                test_index = [x for x in list(range(94)) if x not in train_index]
                X_train, X_test, y_train, y_test= index_train_test_split(X, y, train_index,test_index)
                print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape))

                train_size = X_train.shape[0]/(X_train.shape[0]+X_test.shape[0])
                print('train size', train_size)

                if train_size>0.25:
                    X_train = X_train[::4]
                    X_test = X_test[::4]
                    y_train = y_train[::4]
                    y_test = y_test[::4]

                automl = autosklearn.regression.AutoSklearnRegressor(
                        time_left_for_this_task=30,#60*time_limit,
                        per_run_time_limit=2,#30*time_limit,
                        include_estimators = ['extra_trees'],
                        )

                automl.fit(X_train, y_train)

                print(automl.show_models())

                predictions = automl.predict(X_test)

                r2_score = sklearn.metrics.r2_score(y_test, predictions)
                rmse = sklearn.metrics.mean_squared_error(y_test, predictions)**0.5

                r2_scores.append(r2_score)
                rmses.append(rmse)

                print("[*]R2 score:", r2_score,file=logfile)
                print("[*]rmse:", rmse,file=logfile)

                print("[*]analysis:")
                print(r2_scores,'\n',rmses)


                s = pickle.dumps(automl)
                with open('./point_models/model_pt'+str(len(train_index))+'.pkl', 'wb') as f:
                    f.write(s)
                print('[*]save model!')

                model_index = model_index + 1

                logfile.close()

        r2_scores = np.array(r2_scores)
        rmses = np.array(rmses)

        temp_r2_mean = np.mean(r2_scores)
        temp_r2_std = np.std(r2_scores)

        temp_rmses_mean = np.mean(rmses)
        temp_rmses_std = np.std(rmses)

        r2_mean.append(temp_r2_mean)
        r2_std.append(temp_r2_std)

        rmse_mean.append(temp_rmses_mean)
        rmse_std.append(temp_rmses_std)
        print('\n\n\n','[*]calculate:',r2_mean,r2_std,rmse_mean,rmse_std,'\n\n\n')

    print('all done')
#load data

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

'''
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
'''

if __name__ == "__main__":
    main()
