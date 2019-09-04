import numpy as np
import scipy
import random
from klampt.math import vectorops as vo
def load_data(exp_path, probe_type='point', Xtype='loc',ytype='f',logfile=None):
    
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
            
        elif probe_type == 'ellipse':
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
                displacement.append(l2[3])
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
            elif Xtype == 'loc_color_cur':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 np.tile(colors[i],(num_dis,1)), 
                                 np.tile(curvatures[i],(num_dis,1)), 
                                 displacement))

            Y_i = np.array(force_normal,ndmin=2).T
            X.append(X_i)
            Y.append(Y_i)

        elif probe_type == 'line':
            num_dis = len(displacement)
            #print('---load %d displacement'%num_dis)
            displacement = np.resize(np.array(displacement),(num_dis,1)) 
            if Xtype == 'loc_color_cur':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 np.tile(colors[i],(num_dis,1)), 
                                 np.tile(curvatures[i],(num_dis,1)), 
                                 displacement))
            Y1_i = np.array(force_normal,ndmin=2).T
            Y2_i = np.array(torque_normal,ndmin=2).T
            if ytype == 'f':
                Y_i = Y1_i
            elif ytype == 'ft':
                Y_i=np.hstack((Y1_i,Y2_i))
            X.append(X_i)
            Y.append(Y_i)
            
        elif probe_type == 'circle':
            num_dis = len(displacement)
            #print('---load %d displacement'%num_dis)
            displacement = np.resize(np.array(displacement),(num_dis,1)) 
            if Xtype == 'loc_color_cur':
                X_i = np.hstack((np.tile(points[i],(num_dis,1)), 
                                 np.tile(colors[i],(num_dis,1)), 
                                 np.tile(curvatures[i],(num_dis,1)), 
                                 displacement))
            Y1_i = np.array(force_normal,ndmin=2).T
            Y2_i = np.array(torque_normal,ndmin=2).T
            if ytype == 'f':
                Y_i = Y1_i
            elif ytype == 'ft':
                Y_i=np.hstack((Y1_i,Y2_i))
            X.append(X_i)
            Y.append(Y_i)

    return X,Y

def cal_line_data(exp_path):

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