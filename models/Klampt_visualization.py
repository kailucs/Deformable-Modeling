from klampt.io import loader
from klampt import vis
from klampt import *
import data_loader
import open3d
import numpy as np
#pcd = PointCloud()
#pcd.propertyNames.append('rgb')
#pcd.propertyNames.append('r')
#pcd.propertyNames.append('g')
#pcd.propertyNames.append('b')
#pcd.propertyNames.append('a')

exp_N = '3'
exp_path='../data_final/exp_' + exp_N + '/'
exp_path_2 = '../data_final/exp_' + exp_N + '_debiased/'
#load visual model
#pcd = data_loader.load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
#open3dPcd = open3d.PointCloud()
#xyz = []
#rgb = []
#normals = []
#for ele in pcd:
#    xyz.append(ele[0:3])
#    rgb.append(ele[3:6])
#    normals.append(ele[6:9])
#open3dPcd.points = open3d.Vector3dVector(np.asarray(xyz,dtype=np.float32))
#open3dPcd.colors = open3d.Vector3dVector(np.asarray(rgb,dtype=np.float32))
#open3dPcd.normals = open3d.Vector3dVector(np.asarray(normals,dtype=np.float32))
#open3d.io.write_point_cloud("testPcd.pcd", open3dPcd)

#a = open3d.io.read_point_cloud("testPcd.pcd")
#open3d.visualization.draw_geometries(
#        [a])

#geom=Geometry3D()
#geom.loadFile("testPcd.pcd")

klamptPcd = loader.loadGeometry3D("testPcd.pcd")
#ointCloudObject = klamptPcd.get
#properties = []
#for ele in pc:
#
#    pcd.addPoint(ele[0:3])
#    #properties = properties + ele[3:6]
#    properties = properties + [0xffffff]
#pcd.setProperties(properties)
#pcd.properties.append([1,0.5,0.1,1,1,0.5,0.1,1])
#world = WorldModel()
#world.add('PointCloud',PointCloud)
#vis.add("world",world)
#geom = Geometry3D(pcd)

vis.add('PointCloud',klamptPcd)
vis.hideLabel('PointCloud')
vis.run()
