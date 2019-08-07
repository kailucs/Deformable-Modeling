# %load analyse.py
import numpy as np
import autosklearn.regression
import sklearn.model_selection
import sklearn.datasets
import sklearn.metrics
import scipy
import pickle
import random
import matplotlib.pyplot as plt

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

def my_train_test_split(X,y,num_point=1,train_size=0.8,use_all=False):
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

def main():
    X,y = load_data('./probePcd.txt','.') #note: is list
    
    X,y = my_train_test_split(X,y,train_size=1,use_all=True)
    Xy = np.hstack((X,y))

    set_displacement = 0.003
    Xy = Xy[Xy[:,3]==set_displacement]
    X_test = Xy[:,0:4]
    y_test = Xy[:,4]
    print(Xy.shape)
    print('load ori data with displacement: %f'%set_displacement)

    with open('model_7.pkl', 'rb') as f:
        s2 = f.read()
        automl = pickle.loads(s2)
        predictions = automl.predict(X_test)
    print('[*]load model!')

    errors = np.resize(np.fabs(predictions-y_test),(110,1))
    print(y_test)
    print(predictions)
    print(errors)
    Xe = np.hstack((X_test,errors))
    

    # test
    num_train_points=range(11,99,11)
    r2_scores=[0.6967633043173058,0.7684077304309724,0.7793326420064322,0.8112461987394535,
                0.8275108633318626,0.8524415416195835,0.8367747354910278,0.9042793639284351]

    cm = plt.cm.get_cmap('RdYlBu')
    sc = plt.scatter(Xe[:,0], Xe[:,1], c=Xe[:,4], vmin=0, vmax=np.max(Xe[4]), s=1, cmap=cm)
    plt.colorbar(sc)
    plt.show()

if __name__ == "__main__":
    main()

'''    
X_train, X_test, y_train, y_test = \
    sklearn.model_selection.train_test_split(X, y, random_state=1, test_size=0.2, )
print('[*]dataset loaded! Train: %s, Test: %s'%(X_train.shape,X_test.shape))
'''

'''
with open('model_1.pkl', 'rb') as f:
    s2 = f.read()
    automl = pickle.loads(s2)
    predictions = automl.predict(X_test)
print('[*]load model!')

print(automl.show_models())
predictions = automl.predict(X_test)
print("[*]R2 score:", sklearn.metrics.r2_score(y_test, predictions))

exit()
'''
