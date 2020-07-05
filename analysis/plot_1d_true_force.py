# this file is originally in exp_3_debiased folder
# plot the force-distance curve

import numpy as np
import autosklearn.regression
import sklearn.model_selection
import sklearn.datasets
import sklearn.metrics
import scipy
import pickle
import random
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 100

def load_data(point_path, force_path, probe_type='point', Xtype='loc'):
    
    points=[]
    colors=[]
    normals=[]
    curvatures=[]
    theta = []
    
    # load ori data
    dataFile=open(point_path,'r')

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

def my_train_test_split2(X,y,num_point=1,train_size=0.8,use_all=False):
    num_point = len(X)
    train_index = random.sample(range(num_point),int(train_size*num_point))
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
    
    if use_all == False:
        flag = 0
        for i in test_index:
            if flag==0:
                X_test = X[i]
                y_test = y[i]
                flag = 1
            else:
                X_test = np.vstack((X_test,X[i]))
                y_test = np.vstack((y_test,y[i]))
        
    if use_all == False:
        return X_train, X_test, y_train, y_test
    else:
        return X_train, y_train

def main(point):
    for point_index in range(1):
        X,y = load_data('./probePcd.txt','.',Xtype='loc') #note: is list

        X,y = my_train_test_split2(X,y,train_size=1,use_all=True)
        Xy = np.hstack((X,y))
        print(Xy.shape)
        select_loc = X[point]

        Xy = Xy[Xy[:,0]==select_loc[0]]
        Xy = Xy[Xy[:,1]==select_loc[1]]
        Xy = Xy[Xy[:,2]==select_loc[2]]

        X_test = Xy[:,0:4]
        y_test = Xy[:,4]
        displacement = Xy[:,3]
        print(Xy.shape)
        
        
        #with open('./models_dense/model_pt10.pkl', 'rb') as f:
         #   s2 = f.read()
          #  automl = pickle.loads(s2)
           # predictions = automl.predict(X_test)
       # print('[*]load model!')
        
        
        #errors = np.resize(np.fabs(predictions-y_test),(110,1))
        #print(y_test)
        #print(predictions)
        #print(errors)
        #print(automl.show_models())
        #automl.sprint_statistics()


        plt.plot(displacement*1000,y_test,color='b',marker='o',markersize=0.1,label='prediction')
        #plt.plot(displacement,predictions,color='r',marker='o',markersize=0.1,label='prediction')
        plt.xlabel('Displacement(mm)',fontsize=25)
        plt.ylabel('Force(N)',fontsize=25)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        
        plt.savefig('./force_curve_fig/tmp_'+str(point)+'.png')
        plt.show()
        #plt.show()
    
if __name__ == "__main__":
    for i in range(10):
        main(i)