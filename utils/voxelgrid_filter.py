import time
import numpy as np
import cv2
from copy import deepcopy
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
import math
import random
import matplotlib.pyplot as plt
from scipy import signal
from open3d import *

def voxelgrid_downsample(oripcl,voxel_size,type='middle',with_normals=True):
    """
    this is voxelgrid filter function.
    input: 
        PointCloud pcl
        voxel_size
        filter tye
    output:
        PointCloud pcl
    """
    final_pcl = PointCloud()

    xyz = np.array(oripcl.points)
    index = range(len(oripcl.points))
    [xmax,ymax,zmax] = [np.max(xyz[:,0]),np.max(xyz[:,1]),np.max(xyz[:,2])]
    [xmin,ymin,zmin] = [np.min(xyz[:,0]),np.min(xyz[:,1]),np.min(xyz[:,2])]

    voxel_index_x = int((xmax-xmin)/voxel_size[0])+1
    voxel_index_y = int((ymax-ymin)/voxel_size[1])+1
    voxel_index_z = int((zmax-zmin)/voxel_size[2])+1

    voxel_boxes = [[] for i in range(voxel_index_x*voxel_index_y*voxel_index_z)]
    
    #print(voxel_index_x,voxel_index_y,voxel_index_z)
    if with_normals == False:
        oripcl.normals = Vector3dVector(np.tile(np.array([0,0,1]),(len(oripcl.points),1)))	

    for [pt,color,normal] in zip(oripcl.points,oripcl.colors,oripcl.normals):
        x = int((pt[0]-xmin)/voxel_size[0])+1
        y = int((pt[1]-ymin)/voxel_size[1])+1
        z = int((pt[2]-zmin)/voxel_size[2])+1
        #print(x,y,z)
        voxel_boxes[(x-1)*voxel_index_y*voxel_index_z+(y-1)*voxel_index_z+z-1].append([pt[0],pt[1],pt[2],color[0],color[1],color[2],normal[0],normal[1],normal[2]])
    
    #
    index_box = []
    for x in range(voxel_index_x):
        for y in range(voxel_index_y):
            for z in range(voxel_index_z):
                index_box.append([x+1,y+1,z+1])
    
    final_xyzrgbn = []
    voxel_boxes_index = 0
    for voxel_box in voxel_boxes:
        if len(voxel_box) == 0:
            pass
            '''
            [x,y,z] = index_box[voxel_boxes_index]
            for around_index in [[1,0],[-1,0],[0,1],[0,-1]]: #4-connect
                tmp_x = x+around_index[0]
                tmy_y = y+around_index[1]
                tmp_z = z
                around_index = (tmp_x-1)*voxel_index_y*voxel_index_z+(tmy_y-1)*voxel_index_z+tmp_z-1
                if len(voxel_boxes[around_index]) != 0
            '''
        else:
            if type == 'middle':
                middle_index = int(len(voxel_box)/2)
                
                tmp_xyzrgbn = np.array(voxel_box)
                set_xyzrgbn = sorted(tmp_xyzrgbn, key=lambda x: (x[0], x[1]))
                final_xyzrgbn.append(set_xyzrgbn[middle_index].tolist())
            elif type == 'zmin':
                zmin_index = 0
                
                tmp_xyzrgbn = np.array(voxel_box)
                set_xyzrgbn = sorted(tmp_xyzrgbn, key=lambda x: (x[2]))
                final_xyzrgbn.append(set_xyzrgbn[zmin_index].tolist())
            elif type == 'special':
                zmin_index = 0
                z_displacement = 0.001

                tmp_xyzrgbn = np.array(voxel_box)
                set_xyzrgbn = sorted(tmp_xyzrgbn, key=lambda x: (x[2]))
                set_xyzrgbn = np.array(set_xyzrgbn)

                min_z = set_xyzrgbn[zmin_index][2]

                set_xyzrgbn = set_xyzrgbn[set_xyzrgbn[:,2]<(min_z+z_displacement)]

                for index in range(set_xyzrgbn.shape[0]):
                    final_xyzrgbn.append(set_xyzrgbn[index].tolist())
        
        voxel_boxes_index = voxel_boxes_index + 1
        
    final_xyzrgbn = np.array(final_xyzrgbn)
    final_xyzrgbn = sorted(final_xyzrgbn, key=lambda x: (x[0], x[1])) #return a list with turple
    final_xyzrgbn = np.array(final_xyzrgbn)

    final_pcl.points = Vector3dVector(final_xyzrgbn[:,0:3])
    final_pcl.colors = Vector3dVector(final_xyzrgbn[:,3:6])
    final_pcl.normals = Vector3dVector(final_xyzrgbn[:,6:9])

    return final_pcl
        