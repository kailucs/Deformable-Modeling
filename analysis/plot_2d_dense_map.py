# this file is originally in exp_3_debiased folder
# plot predict dense map with true force point.
# show the changes of learning process from 1 point to 10 points

import numpy as np
import autosklearn.regression
import sklearn.model_selection
import sklearn.datasets
import sklearn.metrics
import scipy
import pickle
import random
import matplotlib.pyplot as plt
from train_config import *
from data_loader import normalize_points
import glob
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 100
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = (6,8)
import time

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
        colors.append(l2[0:3])
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

def load_pcd(path, pcdtype='xyzrgbn'):
    points=[]
    normals=[]
    normal_theta=[]
    theta=[]
    pt_index=[]
    lines=[]
    dataFile=open(path,'r')

    for line in dataFile:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        lines.append(l2)
        points.append(l2[0:3])
        normals.append(l2[6:9])
        if pcdtype == 'xyzrgbntheta':
            normal_theta.append(l2[10:13])
            theta.append(l2[13])
            pt_index.append(l2[14])
    dataFile.close()
    print('---------------------pcd loaded -----------------------------')
    if pcdtype == 'xyzrgbn':
        return points, normals
    elif pcdtype == 'xyzrgbntheta':
        return points, normals, normal_theta, theta, pt_index
    elif pcdtype == 'return_lines':
        return lines
    
def main(point_num):
    X_poke,y_poke = load_data('./probePcd.txt','.') #note: is list
    
    X = load_pcd('./originalPcd.txt',pcdtype='return_lines') #note: is list
    X = np.array(X)
    print(X.shape)
    X = X[:,0:3]

    set_displacement = -0.002
    
    X = np.hstack((X, np.tile(set_displacement,(X.shape[0],1))))
    X = X[X[:,1]>-0.01]  
    X = normalize_points(X, location_offset[0:3], location_offset[3])

    model_path = './models_dense/model_pt'+str(point_num)+'.pkl'
    print(model_path)
    
    index_cur = train_indexes_dense[0][point_num-1]      
    print(index_cur)
    with open(model_path, 'rb') as f:
        s2 = f.read()
        automl = pickle.loads(s2)
        predictions = automl.predict(X)
    #print('[*]load model and predict at %f s, average: %f s'%(t_e-t_s, (t_e-t_s)/X.shape[0]))
    
    X[:,0] = X[:,0]*location_offset[3]*100 #cm
    X[:,1] = X[:,1]*location_offset[3]*100 #cm
    
    cm = plt.cm.get_cmap('jet')
    #sc = plt.scatter(Xe[:,0], Xe[:,1], c=Xe[:,4], vmin=0, vmax=err_max, s=20, cmap=cm)
    sc = plt.scatter(X[:,0], X[:,1], c=predictions, vmin=0, vmax=1.5, s=4, cmap=cm)
    
    for i in index_cur:
        cur_X_array = X_poke[i]
        cur_y_array = y_poke[i]
        
        #print(cur_X_array[:,3])
        y_true = cur_y_array[cur_X_array[:,3]>set_displacement]
        y_true = y_true[0]
        loc_x = cur_X_array[0,0]*location_offset[3] * 100
        loc_y = cur_X_array[0,1]*location_offset[3]* 100
        
        #colors = plt.cm.jet(y_true)
        
        plt.scatter(loc_x, loc_y, color=plt.cm.jet(y_true/2),s=100,edgecolors='k')#, c=y_true, vmin=0, vmax=3, s=100, cmap=cm)
        #plt.text(loc_x,loc_y,str(i))
        
    cbar=plt.colorbar(sc)
    cbar.ax.set_ylabel('Force (N)', labelpad=30,rotation=270,fontsize=25)
    cbar.ax.tick_params(labelsize=16)
    plt.xlabel('x (cm)', fontsize=25)
    plt.ylabel('y (cm)', fontsize=25)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.axis('auto') 
    plt.savefig('./dense_fig/pt_'+str(point_num)+'.png')
    plt.show()
    
if __name__ == "__main__":
    for i in range(10):
        main(i+1)