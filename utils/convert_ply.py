import math

def run_convert_ply():
	counter = 0
	vertexCounter = 0
	file=open("experiment_data/TSDF_result.ply",'r')
	file2=open("experiment_data/TSDF_converted.ply",'w')
	NofVertices = 67131#124530
	#TODO: convert to config. how to cal?

	for line in file:
		line=line.rstrip()
		#l=[num for num in line.split(' ')]
		#l2=[float(num) for num in l]
		if counter == 7:
			file2.write('property uchar red\n')
			counter = counter + 1
		elif counter == 8:
			file2.write('property uchar green\n')
			counter = counter + 1	
		elif counter == 9:
			file2.write('property uchar blue\n')
			counter = counter + 1
		elif counter < 14:
			file2.write(line)
			file2.write('\n')
			counter = counter + 1
		else:
			if vertexCounter < NofVertices:
				l=[num for num in line.split(' ')]
				l2=[float(num) for num in l]
				file2.write(str(l2[0])+' '+str(l2[1])+ ' ' + str(l2[2])+' ')
				file2.write(str(int(l2[3]*255.0))+' '+str(int(l2[4]*255.0))+ ' ' + str(int(l2[5]*255.0))+' ')
				file2.write(str(l2[6])+'\n')
				vertexCounter = vertexCounter+1
			else:
				file2.write(line)
				file2.write('\n')
	file.close()
	file2.close()
