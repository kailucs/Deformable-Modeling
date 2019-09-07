import numpy as np
import scipy
import random
from klampt.math import vectorops as vo
import pickle

def load_data(exp_path, probe_type='point', Xtype='loc',ytype='f',logfile=None):
    """
    this is model train data load function.
    input: 
        exp_path: 
        probe_type: point, line, circle.
        Xtype: loc, loc_cur, loc_color, loc_color_cur
        ytype: fn ,tn ,fntn, f, t, ft.
        logfile:
    output:
        X:[arr,arr...]
        y:[arr/num,...]   
    """

    points=[]
    colors=[]
    normals=[]
    curvatures=[]
    theta = []
    
    # load ori data
    if probe_type == 'point':
        dataFile=open(exp_path+'probePcd.txt','r')
    elif probe_type == 'line':
        dataFile=open(exp_path+'probePcd_line_theta.txt','r')
    elif probe_type == 'circle':
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
        
    points = np.array(points)
    colors = np.array(colors)
    normals = np.array(normals)
    curvatures = np.array(curvatures)

    # normalize, note colors and normals is already 0~1
    points = normalize_points(points)
    num_point = len(points)
    
    print('[*]load %d points, and normalized'%num_point)
    #if logfile != None:
    #    print('[*]load %d points, and normalized'%num_point,file=logfile)
    
    X=[]
    Y=[]

    for i in range(num_point):
        #load force/torque data
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

        if probe_type == 'line':
            torque_path = exp_path+probe_type+'/torque_'+str(i)+'.txt'
            torque=[]
            torque_normal=[]
            displacement=[]
            theta=[]

            dataFile=open(torque_path,'r')
            for line in dataFile:
                line=line.rstrip()
                l=[num for num in line.split(' ')]
                l2=[float(num) for num in l]
                torque.append(l2[0:3])
                torque_normal.append(l2[3])
                displacement.append(l2[4])
            dataFile.close()
            
        elif probe_type == 'circle':
            torque_path = exp_path+probe_type+'/torque_'+str(i)+'.txt'
            torque=[]
            displacement=[]
            theta=[]

            dataFile=open(torque_path,'r')
            for line in dataFile:
                line=line.rstrip()
                l=[num for num in line.split(' ')]
                l2=[float(num) for num in l]
                torque.append(l2[0:3])
                displacement.append(l2[3])
            dataFile.close()
               
        # final
        # construct X
        num_dis = len(displacement)
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
        elif Xtype == 'loc_color_cur':
            X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                np.tile(colors[i],(num_dis,1)), 
                                np.tile(curvatures[i],(num_dis,1)), 
                                displacement))
        X.append(X_i)

        #construct y
        if ytype == 'fn':
            Y_i = np.array(force_normal,ndmin=2).T
        elif ytype == 'tn':
            Y_i = np.array(torque_normal,ndmin=2).T
        elif ytype == 'fntn':
            Y1_i = np.array(force_normal,ndmin=2).T
            Y2_i = np.array(torque_normal,ndmin=2).T
            Y_i=np.hstack((Y1_i,Y2_i))

        elif ytype == 'f':
            Y_i = np.array(force,ndmin=2)
        elif ytype == 't':
            Y_i = np.array(torque,ndmin=2)
        elif ytype == 'ft':
            Y1_i = np.array(force,ndmin=2)
            Y2_i = np.array(torque,ndmin=2)
            Y_i=np.hstack((Y1_i,Y2_i))
        Y.append(Y_i)
    return X,Y

def cal_line_data(exp_path):
    """
    this is the function to cal line probe start and end.
    input: 
        exp_path: 
    output:
        line_starts
        line_ends
        normals
        line_torque_axes  
    """

    normals=[]
    line_starts=[]
    line_ends=[]
    line_torque_axes=[]

    probeLength = 0.05 #TODO: will move to config

    # load ori data
    dataFile=open(exp_path+'probePcd_line_theta.txt','r')

    for line in dataFile:        
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]

        point = l2[0:3]
        theta = l2[10:13]
        normal = l2[6:9]

        lineTorqueAxes = np.cross(theta,normal)
        lineTorqueAxes = lineTorqueAxes / np.linalg.norm(lineTorqueAxes)

        lineStart = vo.sub(point,vo.mul(theta,0.5*probeLength))
        lineEnd = vo.add(point,vo.mul(theta,0.5*probeLength))

        line_starts.append(lineStart)
        line_ends.append(lineEnd)
        normals.append(normal)
        line_torque_axes.append(lineTorqueAxes)

    dataFile.close()

    return line_starts,line_ends,normals,line_torque_axes

def load_pcd(file_path, pcdtype='xyzrgbn'):
    """
    this is the function to load a pcd/pcd-theta file.
    input: 
        file_path: 
        pcdtype: 'xyzrgbn', 'xyzrgbntheta', 'return_lines'
    output:
        data selected from the pcd file  
    """

    points=[]
    normals=[]
    normal_theta=[]
    theta=[]
    pt_index=[]
    lines=[]

    dataFile=open(file_path,'r')

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

    print('[*]pcd loaded')

    if pcdtype == 'xyzrgbn':
        return points, normals
    elif pcdtype == 'xyzrgbntheta':
        return points, normals, normal_theta, theta, pt_index
    elif pcdtype == 'return_lines':
        return lines

def load_model(model_path):
    """
    This is the function to load model via pickle.
    Note that pickle relies on the env.
    """
    with open(model_path, 'rb') as f:
        s2 = f.read()
        model = pickle.loads(s2)
    print('[*]load model!')
    return model

def my_train_test_split_special(X,y,num_point=1,train_size=0.8,use_all=False):
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

def normalize_points(points, p_min=0, max_range=0):
    if p_min == 0 and max_range == 0:  
        max_range = max([ (np.max(points[:,0])-np.min(points[:,0])) , 
                        (np.max(points[:,1])-np.min(points[:,1])) , 
                            (np.max(points[:,2])-np.min(points[:,2])) ])
        p_min = []
        for i in range(3):
            p_min.append(np.min(points[:,i]))

    for i in range(3):
        points[:,i] = (points[:,i]-p_min[i])/max_range
    
    print('normalize config: ','min: ', p_min, 'max_range: ',max_range)
    return points 

if __name__ == "__main__":
    #test
    X,y=load_data(exp_path='../../data/exp_3_debiased/',
                    probe_type='point',
                    Xtype='loc',
                    ytype='fn')

    X,y = my_train_test_split_special(X,y,train_size=1,use_all=True)
    Xy = np.hstack((X,y))

    set_displacement = 0.002
    Xy = Xy[Xy[:,3]>set_displacement]
    Xy = Xy[Xy[:,3]<set_displacement+0.0005]
    X_test = Xy[:,0:4]
    y_test = Xy[:,4]
    print(Xy.shape)
    print('load ori data with displacement: %f'%set_displacement)

    pcd = load_pcd('../../data/exp_3_debiased/probePcd.txt','return_lines')

    line_starts,line_ends,normals,line_torque_axes =\
        cal_line_data('../../data/exp_3/')

    m = load_model('/home/dactl/Deformable-Modeling-M/data_final/exp_3_debiased/models/model_mutimodels_0.pkl')
    
    y_predict = m.predict(X_test)
    
    print(y_predict.shape)
    #print(X[10],y[10])
    #print(pcd)
    #print(line_starts,line_ends,normals,line_torque_axes)