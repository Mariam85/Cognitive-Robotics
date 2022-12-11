
from tf.transformations import euler_from_quaternion

import numpy as np

#import the created globals file
import globals as g

#########################   SENSOR UTILS    ############################################

def ThetaFromQuant(orientations):
    """
    Transform theta in radians from quanterion 
    """
    #
    xyzw = [ orientations.x, orientations.y, orientations.z, orientations.w]
    (_, _, yaw) = euler_from_quaternion(xyzw)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # from 0 to 2pi
    return yaw

def ThetaFromOdom(odom):
    """"
    Get theta from Odometry
    """
    orientations = odom.pose.pose.orientation
    theta = ThetaFromQuant(orientations) 
    return theta

#convert x and y to map coordinates
def convert_to_map(x,y):
    # x = int(x/g.metadata.resolution)
    # y = int(y/g.metadata.resolution)
    origin_x = int(g.metadata.height/2 - y/g.metadata.resolution)
    origin_y = int(g.metadata.width/2 + x/g.metadata.resolution)
     
    return origin_x,origin_y

def convert_to_map_array(xs,ys):
    origin_xs = (g.metadata.height/2 - ys/g.metadata.resolution).astype(int)
    origin_ys = (g.metadata.width/2 + xs/g.metadata.resolution).astype(int)
    return origin_xs,origin_ys

#convert map coordinates to x and y
def convert_to_xy(x,y):
    x = x*g.metadata.resolution
    y = y*g.metadata.resolution
    return x,y


        
#get x , y and theta from odometry
def get_pose(odom_data):
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    theta = ThetaFromOdom(odom_data)
    return x,y,theta

def get_laser_measuremts(laser_data):
    """
    Returns distances and thier crossponding thetas per laser scan
    """
    distances = np.array([])
    thetas = np.array([])
    for i in range(len(laser_data.ranges)):
        distance = laser_data.ranges[i]
        #discard above max or below min
        distance = laser_data.range_max if distance > laser_data.range_max else (laser_data.range_min if distance < laser_data.range_min else distance)
        distances = np.append(distances,distance)
        #in radians
        thetas = np.append(thetas,i*laser_data.angle_increment)
    return distances,thetas


def get_sensor_data(data):
    """
    Returns odometry and laser scan data
    """
    x,y,theta = get_pose(data)
    distances,thetas = get_laser_measuremts(data)
    return x,y,theta,distances,thetas

def get_sensor_data_as_xy(x,y,theta,distances,thetas):
    """
    Returns x and y of laser scan
    """
    #for each laser scan get x and y
    xs = np.array([])
    ys = np.array([])
    for i in range(len(distances)):
        #find acute angle
        acute = theta + thetas[i] - np.pi/2
        #find x and y
        x1 = x + distances[i]*np.cos(acute)
        y1 = y + distances[i]*np.sin(acute)
        # if acute > np.pi/2:
        #     acute = np.pi - acute
        #     x1 = x - distances[i]*np.cos(acute)
        #     y1 = y + distances[i]*np.sin(acute)
        # elif acute > np.pi:
        #     acute = acute - np.pi
        #     x1 = x - distances[i]*np.cos(acute)
        #     y1 = y - distances[i]*np.sin(acute)
        # elif acute > 3*np.pi/2:
        #     acute = 2*np.pi - acute
        #     x1 = x + distances[i]*np.cos(acute)
        #     y1 = y - distances[i]*np.sin(acute)
        xs = np.append(xs,x1)
        ys = np.append(ys,y1)
    return xs,ys




#################   MAP UTILS   ###############################################


def log_odds(p):
    """
    Returns log odds of p
    """
    return np.log(p/(1-p))

def log_odds_map(map):
    """
    Returns log odds of map
    """
    logMap = np.copy(map)

    for i in range(len(map)):
        logMap[i] = log_odds(map[i])
    return logMap

def p_from_log_odds_map(log_odds_map):
    """
    Returns p from log odds map
    """
    pMap = np.copy(log_odds_map)

    for i in range(len(log_odds_map)):
        for j in range(len(log_odds_map[i])):
            pMap[i][j] = p_from_log_odds(log_odds_map[i][j])
    return pMap


    

def p_from_log_odds(log_odds):
    """
    Returns p from log odds
    """
    return 1/(1+np.exp(-log_odds))

def update_map(x1,y1,xs,ys):
    """
    Updates map with laser scan
    """

    #update map with laser scan
    for i in range(len(xs)):
        x2 = xs[i]
        y2 = ys[i]
        #get all cells between laser scan and robot
        cells = bresenham_line_algo(x1,y1,x2,y2)
        # print(cells)
        #update map with laser scan
        # print("x1,y1,x2,y2",x1,y1,x2,y2)    
        # print("x2,y2",x2,y2)
        #set cell to occupied
        if x2 >= 0 and x2 < g.metadata.height and y2 >= 0 and y2 < g.metadata.width:
            # print(log_odds(g.occupied_threshold))
            g.map_data[x2][y2] += log_odds(g.occupied_threshold)
            # print(g.map_data[x2][y2])
        for cell in cells:
            x = cell[0]
            y = cell[1]
            #if out of bounds continue
            if x < 0 or x >= g.metadata.height or y < 0 or y >= g.metadata.width:
                continue
            #set cell to free
            g.map_data[x][y] += log_odds(g.free_threshold)
    
    #normalise map to be integer between 0 and 100
    g.map.data = normalise_map(p_from_log_odds_map(g.map_data))
    # print(g.map_data)
    # print(g.map.data)
    return


def normalise_map(map):
    """
    Returns normalised map
    """
    #normalise map to be integer between 0 and 100
    #map is a list 
    for i in range(len(map)):
        for j in range(len(map[i])):
            map[i][j] = int(map[i][j]*100)

    return map.flatten().astype(np.int8)

    ########### breshman algorithm ############
def bresenham_line_algo(x1,y1,x2,y2):
    """
    Returns all cells between two points
    """
    #x is row, y is col
    cells = []
    #get all cells between laser scan and robot
    dx = abs(x2-x1)
    dy = abs(y2-y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx-dy
    while True:
        if x1 == x2 and y1 == y2:
            break
        cells.append((x1,y1))
        e2 = 2*err
        if e2 > -dy:
            err = err - dy
            x1 = x1 + sx
        if e2 < dx:
            err = err + dx
            y1 = y1 + sy
    return cells
