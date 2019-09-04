from data_loader import *
from scipy import spatial
from copy import deepcopy

def invKernel(p1,p2,param):
    r = vo.norm(vo.sub(p1,p2))
    return param/(param+r)

#def line_to_point(exp_path):
#load

DEBUGPROJECTEDPTS = False
DEBUGDISPLACEDPTS = False
#exp_path='../../data/exp_3_debaised/'
exp_path='../../data/exp_1/'
exp_path_2 = '../../data/exp_1_debiased/'
#load line geometry data 
lineStarts,lineEnds,lineNormals,lineTorqueAxes = cal_line_data(exp_path)
#load line force torque data 
X,Y = load_data(exp_path, probe_type='line', Xtype='loc_color_cur',ytype='ft',logfile=None)
#load visual model
pcd = load_pcd(exp_path_2+'originalPcd.txt',pcdtype='return_lines')
#pcd = np.array(pcd)

param = 0.03 # in meters
discretization = 0.003 # in meters
#Iterate through different probe locations
num_iter = [23]
for pointNumber in num_iter:

    lineStart0 = lineStarts[pointNumber]
    lineEnd0 =lineEnds[pointNumber]
    lineTorqueAxis = lineTorqueAxes[pointNumber]
    N = 1 + round(vo.norm(vo.sub(lineStart0,lineEnd0))/discretization)
    s = np.linspace(0,1,N)
    lineNormal = lineNormals[pointNumber]
    localXinW = vo.sub(lineEnd0,lineStart0)/np.linalg.norm(vo.sub(lineEnd0,lineStart0))
    localYinW = (lineTorqueAxis)/np.linalg.norm(lineTorqueAxis)
    lineStartinLocal = [0,0]
    lineEndinLocal = [np.dot(vo.sub(lineEnd0,lineStart0),localXinW),
                    np.dot(vo.sub(lineEnd0,lineStart0),localYinW)]
    #################first project the pcd to the plane of the line##############
    #The start point is the origin of the plane...
    projectedPcd = [] 
    #the projected Pcd is in local frame....
    projectedPcdIdxList = []#Idx in the original pcd 
    NofProjectedPoints = 0
    #pcdThatWasProjected = []
    for i in range(len(pcd)):
        p = pcd[i][0:3]
        projectedPt = vo.sub(p,vo.mul(lineNormal,vo.dot(vo.sub(p,lineStart0),lineNormal))) ##world origin
        projectedPt2D = vo.sub(projectedPt,lineStart0)#world origin
        projectedPt2DinLocal = [vo.dot(projectedPt2D,localXinW),vo.dot(projectedPt2D,localYinW)]
        #%make sure point is in the "box" defined by the line
        if ((projectedPt2DinLocal[0]<0.051) and (projectedPt2DinLocal[0] > -0.001)
            and (projectedPt2DinLocal[1]<0.001) and (projectedPt2DinLocal[1]>-0.001)):
            NofProjectedPoints = NofProjectedPoints + 1
            projectedPcd.append(projectedPt2DinLocal)
            projectedPcdIdxList.append(i)
            #pcdThatWasProjected.append(p)


    #Create a KDTree for searching
    projectedPcdTree = spatial.KDTree(projectedPcd)

    ###############Find the corresponding point on the surface to the line############
    surfacePtsAll = []# %These are the surface pts that will be displaced.
    #%part of the probe not in contact with the object...
    #average 3 neighbors
    NofN = 3  
       
    for i in range(int(N)):
        tmp =vo.mul(vo.sub(lineEnd0,lineStart0),s[i])#%the point on the line, projected 
        linePt = [vo.dot(tmp,localXinW),vo.dot(tmp,localYinW)]
        d,Idx = projectedPcdTree.query(linePt[0:2],k=NofN)
        #We might end up having duplicated pts...
        #We should make sure that the discretization is not too fine..
        #or should average a few neighbors
        surfacePt = [0]*10
        for j in range(NofN):
            surfacePt = vo.add(surfacePt,pcd[projectedPcdIdxList[Idx[j]]][0:10])
        surfacePt = vo.div(surfacePt,NofN)
        surfacePtsAll.append(surfacePt) #position in the global frame..
    
    
    ############# Go through a list of displacements
    queryDList = [0.012]
    totalFinNList = []
    #for queryD = -0.003:0.001:0.014
    for queryD in queryDList:
        print(queryD)
        lineStart = vo.sub(lineStart0,vo.mul(lineNormal,queryD))
        lineEnd = vo.sub(lineEnd0,vo.mul(lineNormal,queryD))
        lineCenter = vo.div(vo.add(lineStart,lineEnd),2)
        localXinW = vo.div(vo.sub(lineEnd,lineStart),vo.norm(vo.sub(lineEnd,lineStart)))
        localYinW = vo.div(lineTorqueAxis,vo.norm(lineTorqueAxis))

        ####calculate the nominal displacements
        surfacePts = []; #These are the surface pts that will be displaced...
        nominalD = []; #Column Vector..
        for i in range(int(N)):
            linePt = vo.add(vo.mul(vo.sub(lineEnd,lineStart),s[i]),lineStart)
            surfacePt = surfacePtsAll[i][0:3]
            normal = surfacePtsAll[i][6:9]
            nominalDisp = -vo.dot(vo.sub(linePt,surfacePt),normal)
            if nominalDisp > 0:
                surfacePts.append(surfacePtsAll[i][0:10])#position in the global frame..
                nominalD.append(nominalDisp)
        originalNominalD = deepcopy(nominalD)
        #print('Deformed Surface Points',surfacePts)
        print('Calculating Actual D....')
        #####Calculate actual displacements
        NofSurfacePts = len(surfacePts)
        originalSurfacePts = deepcopy(surfacePts)
        if NofSurfacePts > 0:
            negativeDisp = True
            while negativeDisp:
                NofSurfacePts = len(surfacePts)
                K = np.zeros((NofSurfacePts,NofSurfacePts))
                for i in range(NofSurfacePts):
                    for j in range(NofSurfacePts):
                        K[i][j] = invKernel(surfacePts[i][0:3],surfacePts[j][0:3],param)
                #print K
                actualD =  np.dot(np.linalg.inv(K),nominalD)
                #print nominalD,actualD
                negativeIndex = actualD < 0
                if np.sum(negativeIndex) > 0:
                    actualD = actualD.tolist()
                    positiveIndex = actualD >= 0
                    #surfacePts = surfacePts[positiveIndex]
                    surfacePts = [surfacePts[i] for i in range(len(surfacePts)) if actualD[i]>=0]
                    nominalD = [nominalD[i] for i in range(len(nominalD)) if actualD[i]>=0]
                else:
                    negativeDisp = False
        
        print originalNominalD
        print nominalD,actualD


