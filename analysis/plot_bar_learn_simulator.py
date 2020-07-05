import numpy as np
import matplotlib.pyplot as plt

data = [[ 66386, 174296,  75131, 577908,  32015],
        [ 58230, 381139,  78045,  99308, 160454],
        [ 89135,  80552, 152558, 497981, 603535],
        [ 78415,  81858, 150656, 193263,  69638],
        [139361, 331509, 343164, 781380,  52269]]

rmse_point_force_true = [0.2853980749171685, 0.742656285395602, 0.3974943368046349, 0.748214442918706, 0.2812494039718233]
rmse_point_force = [0,0,0,0,0]
rmse_point_torque = [0,0,0,0,0]

rmse_line_force = [0.5641557102904055, 2.890557688721938,0.6888629402641337,1.8077946941647385, 0.836418228413565]
rmse_line_torque = [0.037972058889402936, 0.06608916163245086,0.02606816719071163, 0.10086157578457948, 0.03645133616710244]
rmse_cylinder_force = [0.5054318728498662, 3.892519443212705, 0.7021025210071549, 2.27113246342714, 0.9924035955877978]
rmse_cylinder_torque = [0.14009345728820077, 0.15734022256670138, 0.12734037302086476, 0.21240184958131567, 0.12757504298996933]

rmse_line_force = [0.5641557102904055,1.75981885568491,0.6888629402641337,1.7895510057296584, 0.49087674790969416]
rmse_line_torque = [0.038067552565786235,0.09559901358239148,0.02606816719071163,0.11187837675292317, 0.03987053666937059]
rmse_cylinder_force =[0.4774226712759641,3.8160881403615985, 0.7021025210071549, 2.2564013541226084, 0.8872921176919378]
rmse_cylinder_torque = [0.14009176956274771, 0.139155665246031, 0.12734037302086476, 0.2166213128919115, 0.12991804169679233]

rmse_line_force_m = [0.4107513978724102,2.50906210909887,0.4691569364954487,1.3641252292733028,0.6167956802102332]
rmse_line_torque_m = [0.04538060126256089,0.0739904616507208,0.026768072996015507,0.08864182622529142,0.034171617823074846]
rmse_cylinder_force_m = [0.438209274535281,1.9799129700504308,1.1481763916673975,1.0062699628432619,0.46149115816763686]
rmse_cylinder_torque_m = [0.14933499103314885,0.15086430910887547,0.08472900193826832,0.18781009217874722,0.12075069039588389]

# tmp
sum1 = 0
for i in range(5):
    sum1 = sum1 + rmse_cylinder_force[i]
print(sum1/5)

data = [rmse_point_force, rmse_line_force, rmse_line_force_m,
        rmse_cylinder_force,rmse_cylinder_force_m]
data = np.array(data).T.tolist()

data2 = [rmse_point_torque,rmse_line_torque, rmse_line_torque_m,
        rmse_cylinder_torque,rmse_cylinder_torque_m]
data2 = np.array(data2).T.tolist()

columns = ('Point(model)','Line(solver)', 'line(model)', 'cylinder(solver)', 'cylinder(model)')
rows = ['Sloth', 'Vest', 'Lamb', 'Shoe', 'Bird']
labels = ['Point\n(learning)','Line\n(Simulator)', 'Line\n(Learning)', 'Cylinder\n(Simulator)', 'Cylinder\n(Learning)']

values = np.arange(0, 10, 1)
value_increment = 1

values2 = np.arange(0, 1, 0.1)
value_increment2 = 1

# Get some pastel shades for the colors
colors = plt.cm.BuPu(np.linspace(0, 1, len(rows)))
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
#colors = ['C0', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6']
n_rows = len(data)

index = np.arange(len(columns)) + 1.0
bar_width = 0.4

# Initialize the vertical-offset for the stacked bar chart.
y_offset = np.zeros(len(columns))
y_offset2 = np.zeros(len(columns))
# Plot bars and create text labels for the table
cell_text = []
# add to 2 y axis
fig, ax = plt.subplots(1,1)
ax2 = ax.twinx()

for row in range(n_rows):
    print(row)
    ax.bar(index, data[row], bar_width, color=colors[row], bottom=y_offset,ec='k',lw=1)
    y_offset = y_offset + data[row]
    cell_text.append(['%1.6f' % x for x in data[row]])

y_offset = 0
for row in range(n_rows):
    print(row)
    ax.bar(1.2, rmse_point_force_true[row], bar_width, color=colors[row], bottom=y_offset,ec='k',lw=1)
    y_offset = y_offset + rmse_point_force_true[row]
    #cell_text.append(['%1.6f' % x for x in data[row]])
    
for row in range(n_rows):
    ax2.bar(index+0.4, data2[row], bar_width, color=colors[row], bottom=y_offset2,ec='k',lw=1,hatch='/')
    y_offset2 = y_offset2 + data2[row]
    cell_text.append(['%1.6f' % x for x in data2[row]])

# Reverse colors and text labels to display the last value at the top.
#colors = colors[::-1]
cell_text.reverse()

# Add a table at the bottom of the axes
'''
the_table = ax.table(cellText=cell_text,
                      rowLabels=rows,
                      rowColours=colors,
                      colLabels=columns,
                      loc='bottom')
'''
# Adjust layout to make room for the table:
plt.subplots_adjust(left=0.2, bottom=0.2)

ax.set_ylabel('Force RMSE (N)',fontsize=12)
ax2.set_ylabel('Torque RMSE (N·m)',fontsize=12)

x = np.arange(1.2,6.2,1)
ax.set_xticks(x)
ax.set_xticklabels(labels,fontsize=12)

ax.set_yticks(values * value_increment, ['%d' % val for val in values])
ax2.set_yticks(values2 * value_increment, ['%d' % val for val in values2])

#ax.set_xticks([])
#plt.title('Line/Cylinder Analysis')

# legend
ax.bar(0, 0, 0, bottom=0,ec='k',lw=1,color='w',label='Force')
ax.bar(0, 0, 0, bottom=0,ec='k',lw=1,color='w',hatch='/',label='Torque')

ax.legend(loc='upper left',fontsize=12)


for i in range(5):
    ax2.bar(0, 0, 0, bottom=0,ec='k',lw=1,color=colors[i],label=rows[i])

# Shrink current axis's height by 10% on the bottom
box = ax2.get_position()
ax2.set_position([box.x0, box.y0 + box.height * 0.1,
                 box.width, box.height * 0.9])
# Put a legend below current axis
ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18),
          fancybox=True, shadow=False, ncol=5)

plt.show()
