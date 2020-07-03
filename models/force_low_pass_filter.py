from data_loader import *
from scipy import signal
import matplotlib.pyplot as plt
import copy
import os
import shutil

def data_filter(exp_path, probe_type='point', Xtype='loc',ytype='f',num_point=0):
    shutil.rmtree(exp_path+probe_type+'_filter', ignore_errors=True)
    os.mkdir(exp_path+probe_type+'_filter')
    for i in range(num_point):
        #load force/torque data
        force_path = exp_path+probe_type+'/force_'+str(i)+'.txt'
        new_force_path = exp_path+probe_type+'_filter'+'/force_'+str(i)+'.txt'
        
        force=[]
        torque=[]
        force_normal=[]
        torque_normal=[]
        displacement=[]

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
            dataFile=open(torque_path,'r')
            for line in dataFile:
                line=line.rstrip()
                l=[num for num in line.split(' ')]
                l2=[float(num) for num in l]
                torque.append(l2[0:3])
                torque_normal.append(l2[3])
            dataFile.close()
            
        elif probe_type == 'ellipse':
            torque_path = exp_path+probe_type+'/torque_'+str(i)+'.txt'
            dataFile=open(torque_path,'r')
            for line in dataFile:
                line=line.rstrip()
                l=[num for num in line.split(' ')]
                l2=[float(num) for num in l]
                torque.append(l2[0:3])
                displacement.append(l2[3])
            dataFile.close()
        
        #to np
        force=np.array(force,ndmin=2)
        torque=np.array(torque,ndmin=2)
        force_normal=np.array(force_normal,ndmin=2).T
        torque_normal=np.array(torque_normal,ndmin=2).T
        displacement=np.array(displacement)
        #filter
        Wn=0.01
        [b,a]=signal.butter(5,Wn,'low')
        
        for i in range(3):
            tmp_filteredForces=signal.filtfilt(b,a,force[:,i].T,padlen=150)
            if i == 0:
                filteredForces = np.array(tmp_filteredForces,ndmin=2).T
                print(filteredForces.shape)
            else:
                filteredForces = np.hstack((filteredForces,np.array(tmp_filteredForces,ndmin=2).T))
        
        if probe_type == 'line' or probe_type == 'ellipse':
            for i in range(3):
                tmp_filteredTorques=signal.filtfilt(b,a,torque[:,i].T,padlen=150)
                if i == 0:
                    filteredTorques = tmp_filteredTorques.T
                else:
                    filteredTorques = np.hstack((filteredTorques,tmp_filteredTorques.T))
        
        filtered_force_normal=signal.filtfilt(b,a,force_normal.T,padlen=150)
        
        if probe_type == 'line':
            filtered_torque_normal=signal.filtfilt(b,a,torque_normal.T,padlen=150)
        
        #filtered_force_normal = filtered_force_normal.T
        
        print(filtered_force_normal.shape)
        new_dataFile=open(new_force_path,'w+')
        for i in range(displacement.shape[0]):
            new_dataFile.write(str(filteredForces[i,0])+' '+str(filteredForces[i,1])+' '+str(filteredForces[i,2])+' ')
            new_dataFile.write(str(filtered_force_normal[0,i])+' '+str(displacement[i])+'\n')
        new_dataFile.close()
        
    return displacement, filtered_force_normal

d,fn = data_filter('./', probe_type='point', Xtype='loc',ytype='tn',num_point=16)
plt.plot(d*1000,fn.T,color='b',marker='o',markersize=0.1)
plt.show()