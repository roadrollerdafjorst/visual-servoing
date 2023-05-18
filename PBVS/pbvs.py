import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
from PIL import Image
from detector import servoing
from transforms import getpv, transformAxes, _2dto3D_coordinateconvert_


i = 25
def imageConversion(arr, h, w):
    width, height = w, h
    global i
    pixels = []
    for x in range(height):
        for y in range(width):
            r, g, b, a = arr[x][y][0], arr[x][y][1], arr[x][y][2], arr[x][y][3]
            # print(r,g,b,a)
            pixels.append((r, g, b, a))

    # Create a new image from the pixel values
    img = Image.new('RGBA', (width, height))
    img.putdata(pixels)
    # img.save(f"output_pbvs.png")
    i += 1
    return np.array(img)

def main():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/4])
    boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)
    textureId = p.loadTexture("qrcode.png")
    orn = [0, 0, 0]
    
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/4])
    obstacleId = p.loadURDF("cube_small.urdf", [cubeStartPos[0], cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    
    p.changeVisualShape(objectUniqueId=obstacleId,
                     linkIndex=-1, 
                     textureUniqueId=textureId)
    useRealTimeSimulation = 1
    #print(base_link_id)
    if (useRealTimeSimulation):
        p.setRealTimeSimulation(1)

    sleep(1)
    i = 0
    
    if (useRealTimeSimulation):
        
        basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        cubePos, cubeOrn = p.getBasePositionAndOrientation(obstacleId)
        rot_matrix = p.getMatrixFromQuaternion(baseOrn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 0, 1) # y-axis
        # Rotated vectors
        basePos2 = [basePos[0], basePos[1], basePos[2]+0.7]
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(basePos2, basePos + 2 * camera_vector, up_vector)
        sleep(1)
        
        p.stepSimulation()
        width, height = 800, 600
        
        fov = 90


        width, height = 800, 600
        aspect = width / height
        nearVal = 0.01
        farVal = 100
        projection_matrix = p.computeProjectionMatrixFOV(
            fov, aspect, nearVal, farVal
        )
        k = p.getCameraImage(width, height,view_matrix,projection_matrix)
        arr = np.array(k[2])
        arr = imageConversion(arr, height, width)
        points = servoing(arr)

        newpoints = _2dto3D_coordinateconvert_(points, 50, k[3])

        rot_matrix = np.linalg.inv(rot_matrix)
        transform = np.zeros((4, 4))
        for i in range(3):
            for j in range(3):
                transform[i][j] = rot_matrix[i][j]

        basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        transform[0][3] = basePos[0]
        transform[1][3] = basePos[1]
        transform[2][3] = basePos[2]
        transform[3][3] = 1

        tCamRobot = np.zeros((4, 4))
        tCamRobot[0][2] = 1
        tCamRobot[1][0] = -1
        tCamRobot[2][1] = -1
        tCamRobot[0][3] = basePos[0]
        tCamRobot[1][3] = basePos[1]
        tCamRobot[2][3] = basePos[2]+.7
        tCamRobot[3][3] = 1

        for i in range(len(newpoints)):
            newpoints[i] = transformAxes(tCamRobot, newpoints[i])
        
        print(newpoints)

        for i in range(len(newpoints)):
            newpoints[i] = transformAxes(transform, newpoints[i])
        
        print(newpoints)

        pv = getpv(newpoints)

        centre = [0, 0, 0]
        for i in range(3):
            for j in range(4):
                centre[i] += newpoints[j][i]
            centre[i] /= 4
        
        ln = np.linalg.norm(pv)
        newPos = [0, 0, 0]

        for i in range(2):
            newPos[i] = 0.2*pv[i]/ln + centre[i]
        
        print(newPos)
        velocity = [0, 0, 0]
        basePos, _ = p.getBasePositionAndOrientation(boxId)
        ln = np.linalg.norm([basePos[i]-newPos[i] for i in range(3)])
        time = ln/(0.2)
        for i in range(2):
            velocity[i] = (basePos[i]-newPos[i])/(time)
        print(time)
        # time /= 2
        while time>0:
            time -= 1
            basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
            cubePos, cubeOrn = p.getBasePositionAndOrientation(obstacleId)
            rot_matrix = p.getMatrixFromQuaternion(baseOrn)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            # Initial vectors
            init_camera_vector = (1, 0, 0) # z-axis
            init_up_vector = (0, 0, 1) # y-axis
            # Rotated vectors
            basePos2 = [basePos[0], basePos[1], basePos[2]+0.7]
            camera_vector = rot_matrix.dot(init_camera_vector)
            up_vector = rot_matrix.dot(init_up_vector)
            view_matrix = p.computeViewMatrix(basePos2, basePos + 2 * camera_vector, up_vector)
            sleep(1)
            
            p.stepSimulation()
            width, height = 800, 600
            
            fov = 90


            width, height = 800, 600
            aspect = width / height
            nearVal = 0.01
            farVal = 100
            projection_matrix = p.computeProjectionMatrixFOV(
                fov, aspect, nearVal, farVal
            )
            k = p.getCameraImage(width, height,view_matrix,projection_matrix)

            basePos, _ = p.getBasePositionAndOrientation(boxId)
            basePos = list(basePos)
            for i in range(3):
                basePos[i] += velocity[i]
            print(basePos)
            p.resetBasePositionAndOrientation(boxId, basePos, baseOrn)
            p.stepSimulation()
            sleep(.5)
        print("done")
        sleep(20)


if __name__ == "__main__":
    main()