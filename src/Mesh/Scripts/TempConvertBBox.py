
import numpy as np

def print_8_point_bbox(bb):
    for i in range(2):
        for j in range(2):
            for k in range(2):
                point = [
                    bb[i, 0], bb[j, 1], bb[k, 2]
                ]
                
                print( '%f, %f, %f' % ( 
                    point[0], point[1], point[2]
                 ) )

if __name__ == '__main__':
    print('OK')

    bboxPair = np.array( [
        [-1.36038, 0.771261, 2.43622],
        [ 1.94941, 1.86765,  5.51075]
    ], dtype=np.float32 )

    print_8_point_bbox(bboxPair)

    print('OK')
    bboxPair = np.array( [
        [ -0.852257, -1.06018, 3.46048 ], 
        [  1.41932,  -0.389353, 5.62708 ]
    ], dtype=np.float32 )

    print_8_point_bbox(bboxPair)
