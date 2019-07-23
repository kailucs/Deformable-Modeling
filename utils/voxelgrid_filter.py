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

def voxelgrid_downsample(oripcl,voxel_size,type='middle'):
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

    voxel_index_x = int((xmax-xmin)/voxel_size)+1
    voxel_index_y = int((ymax-ymin)/voxel_size)+1
    voxel_index_z = int((zmax-zmin)/voxel_size)+1

    voxel_boxes = [[] for i in range(voxel_index_x*voxel_index_y*voxel_index_z)]
    
    #print(voxel_index_x,voxel_index_y,voxel_index_z)

    for [pt,color,normal] in zip(oripcl.points,oripcl.colors,oripcl.normals):
        x = int((pt[0]-xmin)/voxel_size)+1
        y = int((pt[1]-ymin)/voxel_size)+1
        z = int((pt[2]-zmin)/voxel_size)+1
        #print(x,y,z)
        voxel_boxes[(x-1)*voxel_index_y*voxel_index_z+(y-1)*voxel_index_z+z-1].append([pt[0],pt[1],pt[2],color[0],color[1],color[2],normal[0],normal[1],normal[2]])
    
    final_xyzrgbn = []
    for voxel_box in voxel_boxes:
        if len(voxel_box) == 0:
            pass
        else:
            middle_index = int(len(voxel_box)/2)
            
            tmp_xyzrgbn = np.array(voxel_box)
            set_xyzrgbn = sorted(tmp_xyzrgbn, key=lambda x: (x[0], x[1]))
            final_xyzrgbn.append(set_xyzrgbn[middle_index].tolist())
    
    
    final_xyzrgbn = np.array(final_xyzrgbn)
    final_xyzrgbn = sorted(final_xyzrgbn, key=lambda x: (x[0], x[1])) #return a list with turple
    final_xyzrgbn = np.array(final_xyzrgbn)

    final_pcl.points = Vector3dVector(final_xyzrgbn[:,0:3])
    final_pcl.colors = Vector3dVector(final_xyzrgbn[:,3:6])
    final_pcl.normals = Vector3dVector(final_xyzrgbn[:,6:9])

    return final_pcl
        