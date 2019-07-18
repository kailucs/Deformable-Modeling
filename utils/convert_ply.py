import math

def run_convert_ply(config):
	counter = 0
	vertexCounter = 0
	file1=open(config.exp_path+'exp_'+str(config.exp_number)+"/TSDF_result.ply",'r')
	file2=open(config.exp_path+'exp_'+str(config.exp_number)+"/TSDF_converted.ply",'w')

	for line in file1:
		line=line.rstrip()
		#l=[num for num in line.split(' ')]
		#l2=[float(num) for num in l]
		if counter == 3:
			l=[elem for elem in line.split(' ')]
			NofVertices = int(l[2])#124530
			print('[*]Merge PCD: Num of Vertices: %d'%NofVertices)
			file2.write(line)
			file2.write('\n')			
		elif counter == 7:
			file2.write('property uchar red\n')
		elif counter == 8:
			file2.write('property uchar green\n')
		elif counter == 9:
			file2.write('property uchar blue\n')
		elif counter < 14:
			file2.write(line)
			file2.write('\n')
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
		
		counter = counter + 1
		#print('[**]Debug: line ptr: %d'%counter)
	
	file1.close()
	file2.close()
	print('[*]Merge PCD: Done')
