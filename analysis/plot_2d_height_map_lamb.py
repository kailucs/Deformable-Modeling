import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 100

# load data
path = './output.txt'
dataFile=open(path,'r')

loc_x = []
loc_y = []
h = []
for line in dataFile:        
    line=line.rstrip()
    l=[num for num in line.split(' ')]
    l2=[float(num) for num in l]
        
    loc_x.append(l2[0])
    loc_y.append(l2[1])
    h.append(l2[2])
    
dataFile.close()

#plot
plt.figure(figsize=(8,3))

loc_x_fix = [-i*100+21 for i in loc_y]
loc_y_fix = [i*100 for i in loc_x]
h = [i*100 for i in h]

cm = plt.cm.get_cmap('jet')
sc = plt.scatter(loc_x_fix, loc_y_fix, c=h, vmin=0, vmax=6, s=150, cmap=cm, marker='s')
#sc = plt.scatter(loc_x_fix, loc_y_fix, c=h, vmin=0, vmax=6, s=25, cmap=cm,marker='s')
'''
for i in index_cur:
    cur_X_array = X_poke[i]
    cur_y_array = y_poke[i]

    #print(cur_X_array[:,3])
    y_true = cur_y_array[cur_X_array[:,3]>set_displacement]
    #if i == 70 or i ==5:
    if i == 5:
        y_true = [3]
    y_true = y_true[0]
    loc_x = cur_X_array[0,0]*location_offset[3] * 100
    loc_y = cur_X_array[0,1]*location_offset[3]* 100

    #colors = plt.cm.jet(y_true)

    plt.scatter(loc_x, loc_y, color=plt.cm.jet(y_true/3),s=100,edgecolors='k')#, c=y_true, vmin=0, vmax=3, s=100, cmap=cm)
'''
cbar=plt.colorbar(sc)
#cbar.ax.set_ylabel('Height (cm)', labelpad=30,rotation=270,fontsize=24)
cbar.ax.tick_params(labelsize=20)
plt.xlabel('x (cm)', fontsize=24)
#plt.ylabel('y (cm)', fontsize=24)
plt.xticks([2,4,6,8,10,12,14,16,18,20],fontsize=20)
plt.yticks(fontsize=20)
#plt.title('y (cm)',loc='left',fontsize=24)
ax = plt.subplot(111)

ax.annotate('y (cm)',
            xy=(.045, .995), xycoords='figure fraction',
            horizontalalignment='left', verticalalignment='top',
            fontsize=20)
ax.annotate('Height (cm)',
            xy=(.91, .995), xycoords='figure fraction',
            horizontalalignment='right', verticalalignment='top',
            fontsize=20)

plt.axis('auto') 
plt.show()