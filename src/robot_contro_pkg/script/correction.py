import utils

import numpy as np
import globals as g



def generateDistancesCorrection(data):
    g.distancesCorrection = []
    start = data.range_min
    #round start to nearest resolution
    start = round(start/g.metadata.resolution)*g.metadata.resolution
    while start <= data.range_max:
        g.distancesCorrection.append(start)
        start += g.metadata.resolution * 2

    g.thetasCorrection = [i * data.angle_increment for i in range(360)]
    # print(g.distancesCorrection)


def get_predicted_measurements(x,y,robot_theta,map_data,data):
    #get the distance to the closest obstacle for each degree
    #this is done by calculating the distance between the robot and the closest obstacle for each degree
    predicted_measurements = []

    if g.distancesCorrection[0] == -1:
        generateDistancesCorrection(data)
    
    # print ("len of distances correction: ", len(xs))
    #for all the laser scans, get the distance to the closest obstacle, or the max range if no obstacle is found

    for i in range(len(data.ranges)):
 
        theta = i * data.angle_increment
        acute = -robot_theta + theta - np.pi/2
        #move from range min to max in increments of resolution
        #if an obstacle is found, stop and record the distance
        #if no obstacle is found, record the max range
        # print(g.distancesCorrection)

        cos = np.cos(acute)
        sin = np.sin(acute)
        
        for distance in g.distancesCorrection:
            
            #find x and y
            x1 = x + distance*cos
            y1 = -(y - distance*sin)
            #get the map coordinates
            x1,y1 = utils.convert_to_map(x1,y1)

            # print(x1,y1)
            #check if the point is in the map
            if x1 < 0 or x1 >= g.metadata.height or y1 < 0 or y1 >= g.metadata.width:
                g.predicted_measurements[i] = data.range_max
                g.xsPredicted[i] = x1
                g.ysPredicted[i] = y1
                # print("appending max range out of bounds")
                break
            #check if the point is an obstacle
            # print (map_data[x1][y1])
            if map_data[x1][y1] >=  0.5:
                g.predicted_measurements[i] = distance
                
                g.xsPredicted[i] = x1
                g.ysPredicted[i] = y1
                # print("appending distance")
                break
            #if no obstacle is found, record the max range
            if distance >= data.range_max:
                g.predicted_measurements[i] = data.range_max
                
                g.xsPredicted[i] = x1
                g.ysPredicted[i] = y1
                # print("appending max range")
                break
        #if nothing was appended, append the min range
        if g.predicted_measurements[i] == -1:
            g.predicted_measurements[i] = data.range_min
           
            g.xsPredicted[i] = x1
            g.ysPredicted[i] = y1
            # print("appending max range")
    # print("predicted measurements length: ",len(predicted_measurements))
    return predicted_measurements,g.xsPredicted,g.ysPredicted










def correction_stage(x,y,theta,data,cov,twist):

    #if robot is not moving, do not correct
    if twist.linear.x == 0 and twist.angular.z == 0:
        return x,y,theta

    #first step is to calculate the predicted measurements
    #this is done by calculating the distance between the robot and the closest obstacle for each degree

    #get the map data
    map_data = g.map_data

    #x,y are the predicted map coordinates of the robot
    
    #for all the laser scans, get the distance to the closest obstacle, or the max range if no obstacle is found
    get_predicted_measurements(x,y,theta,map_data,data)

    #states and covariance matrix
    #all of size 360
    # statesX = np.array([])
    # statesY = np.array([])
    # statesTheta = np.array([])
    # statesCov = np.array([])
    statesX = np.zeros((360))
    statesY = np.zeros((360))
    statesTheta = np.zeros((360))
    statesCov = np.zeros((360,3,3))


    for i in range(len(data.ranges)):
        #S Matrix 2x1
        S = np.zeros((2,1))
        S[0][0] = g.xsPredicted[i] - x
        S[1][0] = g.ysPredicted[i] - y

        #q Matrix 2x2
        q = np.dot(S,S.T)

        #1/q
        q_inv = 1/q #TODO

        #sigmas
        sigmaX = np.array([S[0]])
        sigmaY = np.array([S[1]])

        
        #H Matrix 2x3

        H = np.array([[sigmaX[0], sigmaY[0], 0],[sigmaY[0], -sigmaX[0], -1]]) 
        H = np.dot(q_inv,H)


        # print(H.shape)
        #Qt Matrix 2x2
        Qt = np.array([[0.02,0],
                      [0,0.02]])
        # print(Qt.shape)

        #St Matrix 2x2
        St = np.dot(H,np.dot(cov,H.T)) + Qt
        # print(St)
        StMatrix = np.array([[St[0][0][0],St[0][1][0]],
                            [St[1][0][0],St[1][1][0]]])

        #if singular matrix, make it non singular
        # if np.linalg.det(StMatrix) == 0:
        #     StMatrix =1/StMatrix
        # else:
        St_inv = np.linalg.inv(StMatrix)
        # print(St_inv)

        #Kt Matrix 3x2
        Kt = np.dot(cov,np.dot(H.T,St_inv))
        # print(Kt.shape)

        # #zt Matrix 2x1
        # print("zt: ",i)
        # print(data.ranges[i])
        # print(g.thetasCorrection[i])
        zt = np.array([data.ranges[i],g.thetasCorrection[i]])

        #zt-hat Matrix 2x1
        zt_hat = np.array([g.predicted_measurements[i],g.thetasCorrection[i]])
        
        #State Matrix 3x1
        state = np.array([x,y,theta])
        
        #State Correction Matrix 3x1
        stateCorrection = state + np.dot(Kt,(zt-zt_hat))
        #make it 3x1
        stateCorrection = np.array([[stateCorrection[0][0]],
                                    [stateCorrection[1][0]],
                                    [stateCorrection[2][0]]])
        # print(stateCorrection.shape)

        #Covariance Correction Matrix 3x3
        covCorrection = cov - np.dot(Kt,np.dot(H,cov))
        # print(covCorrection.shape)


        # #update the state and covariance
        # statesX = np.append(statesX,stateCorrection[0][0])
        # statesY = np.append(statesY,stateCorrection[1][0])
        # statesTheta = np.append(statesTheta,stateCorrection[2][0])
        # statesCov = np.append(statesCov,covCorrection)
        statesX[i] = stateCorrection[0][0]
        statesY[i] = stateCorrection[1][0]
        statesTheta[i] = stateCorrection[2][0]
        statesCov[i] = covCorrection


        # print(i)
        

    statesX = statesX[~np.isposinf(statesX)]
    statesY = statesY[~np.isposinf(statesY)]
    statesTheta = statesTheta[~np.isposinf(statesTheta)]
    # statesCov = statesCov[~np.isposinf(statesCov)]
    statesX = statesX[~np.isneginf(statesX)]
    statesY = statesY[~np.isneginf(statesY)]
    statesTheta = statesTheta[~np.isneginf(statesTheta)]
    # statesCov = statesCov[~np.isneginf(statesCov)]
    x = np.average(statesX)
    y = np.average(statesY)
    # print("theta before: ",theta)
    theta = np.average(statesTheta)
    cov = np.average(statesCov)
    # print("x: ",x)
    # print("y: ",y)
    # print("theta: ",theta)
    # print("cov: ",cov)
    return x,y,theta

        