import numpy as np
import cv2 as cv
from detector import servoing

def getJacobian(u, v, f=1, z=1):
    J = [[-f/z, 0, u/z, u*v/z, -(f + u*u)/f, v], [0, -f/z, v/z, (f+v*v)/f, -u*v/f, -u]]
    return J

fr = cv.imread("output_reference.png")
requiredPos = servoing(fr)

def robotControl(points, k=None):
    global requiredPos
    # print(requiredPos, points)
    error = []
    for i in range(3):
        for j in range(2):
            error.append(requiredPos[i][j] - points[i][j])
    error = np.array(error)
    J = []
    z = [1, 1, 1]
    if k is not None:
        for i in range(3):
            z[i] = k[points[i][1]*800+points[i][0]]
    J += getJacobian(*points[0], z[0])
    J += getJacobian(*points[1], z[1])
    J += getJacobian(*points[2], z[2])
    j = np.array(J)
    J_1 = np.linalg.inv(J)
    velocity = np.matmul(J_1, error)
    
    return velocity