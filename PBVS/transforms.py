import numpy as np

def getpv(newpoints):
    a = [newpoints[2][i]-newpoints[0][i] for i in range(3)]
    b = [newpoints[1][i]-newpoints[0][i] for i in range(3)]
    return np.cross(b, a)

def transformAxes(transform, coordinates):
    if(len(transform[0])==4):
        a = np.ones(4)
    else:
        a = np.ones(3)
    a[:3] = coordinates
    # print(transform, a, coordinates)
    a = np.matmul(transform, a)
    a = a[:3]
    return a

def _2dto3D_coordinateconvert_(points, f=1, k=None):
    newpoints = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    print('@@@@@@  k  @@@@@@/\n',k)
    k=np.ravel(k)
    # print(k.shape)
    # print(points)
    for i in range(4):
        z = k[points[i][1]*800+points[i][0]]
        newpoints[i][0] = points[i][0]*z/f
        newpoints[i][1] = points[i][1]*z/f
        newpoints[i][2] = z/f
    
    return newpoints