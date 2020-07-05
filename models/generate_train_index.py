import random
import numpy as np

def generate_index(num_point,train_size=0.75,select_method='random'):
# TODO: it seems that this has a lot of bugs.
    if select_method=='random':
        train_index = random.sample(range(num_point),int(train_size*num_point))
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='uniform':
        train_index = [int(i*(1.0/train_size)) for i in range(int(train_size*num_point))]
        test_index = [x for x in range(num_point) if x not in train_index]
    elif select_method=='cv':
        num_pt = num_point
        split_set = []
        #for num_pt_in_set in range(10,0,-1):
        for num_pt_in_set in [5,10]:
            point_indexes=list(range(num_pt))
            num_set = int(num_pt/num_pt_in_set)

            tmp_set2 = []
            iter = 0
            for i in range(num_set):
                tmp_set = []
                for ii in range(num_pt_in_set):
                    tmp_index = random.choice(point_indexes)
                    #print(tmp_index)
                    tmp_set.append(tmp_index)
                    point_indexes.remove(tmp_index)
                    print(iter)
                    iter = iter+1
                tmp_set2.append(tmp_set)

            split_set.append(tmp_set2)
            print(split_set)
            #TODO:
    return train_index,test_index

indexes = []
for size in [0.2,0.3,0.5,0.75]:
    indexes_size = []
    for i in range(10):
        train_index,test_index = generate_index(94,size,'random')
        indexes_size.append(train_index)
    indexes.append(indexes_size)

a = [[[11, 66, 91, 82, 88], [75, 43, 14, 49, 9], [56, 74, 36, 46, 3], [1, 33, 68, 41, 63], [53, 78, 15, 44, 6], [27, 79, 28, 85, 17], [61, 72, 64, 16, 76], [92, 70, 58, 71, 4], [21, 83, 84, 12, 20], [57, 60, 19, 55, 42], [22, 52, 48, 47, 31], [25, 7, 32, 8, 45], [90, 37, 34, 81, 18], [13, 77, 35, 62, 29], [93, 86, 67, 87, 38], [0, 10, 39, 5, 69], [30, 24, 51, 80, 59], [54, 26, 40, 65, 73]], [[21, 65, 36, 37, 64, 60, 59, 25, 46, 38], [51, 56, 16, 2, 4, 27, 47, 66, 74, 11], [87, 72, 31, 78, 32, 44, 92, 58, 1, 12], [55, 20, 50, 77, 18, 85, 69, 41, 75, 35], [23, 40, 53, 45, 14, 62, 10, 8, 15, 0], [49, 6, 54, 86, 7, 76, 70, 90, 68, 28], [34, 29, 88, 71, 63, 48, 73, 30, 81, 3], [42, 13, 5, 84, 43, 24, 82, 93, 33, 57], [89, 79, 17, 67, 61, 26, 9, 80, 39, 19]]]
indexes = a + indexes
print(len(indexes))