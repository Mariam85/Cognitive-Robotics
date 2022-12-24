# euler methor for handling quanterions
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#numpy for array operations
import numpy as np

#import the created globals file
import globals as g


#########################   SENSOR UTILS    ############################################

# Get yaw from quaternion orientation
def ThetaFromQuant(orientations):
    """
    Transform theta in radians from quanterion 
    """
    #x,y,z,w orientations obtained from odometry
    xyzw = [ orientations.x, orientations.y, orientations.z, orientations.w]
    #get yaw from quanterion from tf.transformations
    (_, _, yaw) = euler_from_quaternion(xyzw)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # from 0 to 2pi
    return yaw

def quaternionFromEuler(roll, pitch, yaw):
    """
    Transform quanterion from theta in radians
    """
    #get yaw from quanterion from tf.transformations
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion
    

# Get theta from odometry
def ThetaFromOdom(odom):
    """"
    Get theta from Odometry
    """
    orientations = odom.pose.pose.orientation
    theta = ThetaFromQuant(orientations) 
    return theta

#convert x and y to map coordinates for a single point (x,y)
def convert_to_map(x,y):
    #add half of the map to x and subtract from y to get the origin
    #origin_x = row , origin_y = col (use in map_data 2D array)
    origin_x = int(g.metadata.height/2 - y/g.metadata.resolution)
    origin_y = int(g.metadata.width/2 + x/g.metadata.resolution)
     
    return origin_x,origin_y

#convert x array and y array to map coordinates for multiple points (xs,ys)
def convert_to_map_array(xs,ys):
    #add half of the map to x and subtract from y to get the origin
    #origin_x = row , origin_y = col (use in map_data 2D array)
    origin_xs = (g.metadata.height/2 - ys/g.metadata.resolution).astype(int)
    origin_ys = (g.metadata.width/2 + xs/g.metadata.resolution).astype(int)
    return origin_xs,origin_ys
        
#get x , y and theta from odometry
def get_pose(odom_data):
    x = odom_data.pose.pose.position.x
    y = - odom_data.pose.pose.position.y
    theta = ThetaFromOdom(odom_data)
    return x,y,theta

def get_laser_measuremts(laser_data, robot, robot_theta ):
    """
    Returns distances and thier crossponding thetas per laser scan
    """
    #for each laser scan get x and y
    xs = np.array([])
    ys = np.array([])

    for i in range(len(laser_data.ranges)):
        distance = laser_data.ranges[i]
        #discard above max or below min
        distance = laser_data.range_max if distance > laser_data.range_max else (laser_data.range_min if distance < laser_data.range_min else distance)
        theta = i*laser_data.angle_increment

        #find x and y
        #find acute angle
        acute = -robot_theta + theta - np.pi/2
        #find x and y
        x1 = robot.x + distance*np.cos(acute)
        y1 = -(robot.y - distance*np.sin(acute))
        xs = np.append(xs,x1)
        ys = np.append(ys,y1)

    return xs,ys


def get_sensor_data(data):
    """
    Returns odometry and laser scan data
    """
    x,y,theta = get_pose(data)
    xs,ys = get_laser_measuremts(data,data.pose.pose.position,theta)
    return x,y,xs,ys


#################   MAP UTILS   ###############################################


# return log odds of a singulr cell with probability p
def log_odds(p):
    """
    Returns log odds of p
    """
    return np.log(p/(1-p))

# return probability of a singulr cell given log odds
def p_from_log_odds(log_odds):
    """
    Returns p from log odds
    """
    return 1/(1+np.exp(-log_odds))

# update OccupancyGrid map with laser scan
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
        #set cell to occupied if cell is in bounds
        if x2 >= 0 and x2 < g.metadata.height and y2 >= 0 and y2 < g.metadata.width:
            g.map_data[x2][y2] += g.log_odds_occ
            g.map.data[x2*g.metadata.width + y2] = int(p_from_log_odds(g.map_data[x2][y2]) * 100)
        #set cells between laser scan and robot to free
        for cell in cells:
            x = cell[0]
            y = cell[1]
            #if out of bounds continue
            if x < 0 or x >= g.metadata.height or y < 0 or y >= g.metadata.width:
                continue
            #set cell to free
            g.map_data[x][y] += g.log_odds_free
            g.map.data[x*g.metadata.width + y] = int(p_from_log_odds(g.map_data[x][y]) * 100)
            
    return

    ########### breshman algorithm ############
#given 2 points (x1,y1) and (x2,y2) return all cells between them using line drawing algorithm
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

    ############ SLAM Correction #################
    ## Input (mean and covariance (output of prediction stage))
    ## Corrected given measurements
    ## Output mean and covariance

    # x_mean = arr[0], ymean = arr[1] , theta_mean = arr[2]
    # cov --> matrix 3x3
def correction_EKF(x_mean, y_mean ,theta_mean ,cov,xs,ys,distances):
    
    # corrected mean = mean + K (z - z_hat)
    # corrected cov = (I - K H)* cov

    # 1) Get H
    # 2) Get Q
    # 3) Get S -- use H and Q
    # 4) Get K -- use H and S
    # 5) Get Z
    # 6) Get corrected mean -- use K and Z
    # 7) Get corrected cov  -- use K and H

    # 1) Get H
    # -- > get small S (delta) = mean_i_x - mean_t_x .. mean_i_y - mean_t_y
    # -- > = r_t* cos(phi_t + mean_t_theta) .. r_t* sin(phi_t + mean_t_theta)

    #get sensor data
    #xs: x positions of laser scans relative world
    #ys: y positions of laser scans relative world
        
    
    # dummy = np.ones(xs.shape)
    # x_array = x_mean* dummy # 1 x 360
    # y_array = y_mean* dummy # 1x 360
    # theta_array = theta_mean * dummy # 1x 360

    #x_array is a 1x360 array of x_mean
    x_array = np.full(xs.shape, x_mean)
    y_array = np.full(ys.shape, y_mean)
    theta_array = np.full(xs.shape, theta_mean)
        
    # s --> x and y of objects with respect to the car

    s = np.array([xs - x_array,
                    ys - y_array]) # 2x 360 

    # s = np.array(xs - x_array, 
    #                 ys - y_array) # 2x 360

    q_sDot = np.dot(s.T,s) # q+s = squre of matrix s # 360 x 360
    q_s = q_sDot[0] # 1 x 360
    q_r = q_s ** 0.5
    s_vect0 = s[0] 
    s_vect1 = s[1]
    # # z_hat equivalent to h_small 
    # z_hat = np.array(q_r,
    #                     np.arctan2(s_vect1,s_vect0) - theta_array) # 2x 360
    #get z_hat
    z_hat = np.array([q_r,
                        np.arctan2(s_vect1,s_vect0) - theta_array]) # 2x 360
    
    # get the jacobian matrix of h_small (z_hat)
    H=np.array([[-(q_r)*s_vect0, -(q_r)*s_vect1 , 0 , +(q_r)*s_vect0 , +(q_r)*s_vect1 , 0  ],
                [   s_vect1    ,     -s_vect0   ,-q_s,    -s_vect1   ,   +s_vect0     , 0  ],
                [0          ,     0       , 0  ,     0      ,    0        , q_s]])
    
    inv_q = np.linalg.inv(q_sDot) # 1 x 360

    H = np.dot(H,inv_q)

    # 2) Get Q
    Q_t=np.array([[2,0,0],
                    [0,2,0],
                    [0,0,2]])

    # 3) Get S -- use H and Q
    S_t = H.dot(cov).dot(H.T) + Q_t

    # 4) Get K -- use H and S
    inv_S= np.linalg.inv(S_t)
    K_t = cov.dot(H.T).dot(inv_S)
    
    # 5) Get Z -- already done: Z_hat
    # 6) Get corrected mean -- use K and Z
    inpMeanArr = np.array([x_mean,y_mean,theta_mean])
    # try with/ without theta 
    thetas = np.arange(1,360)
    Z_arr = np.array([distances,thetas])
    mean_corrected = inpMeanArr + K_t.dot(Z_arr - z_hat)
    
    # 7) Get corrected cov  -- use K and H
    I = np.eye(3)
    cov_corrected = (I - K_t.dot(H)).dot(cov)

    
    return mean_corrected , cov_corrected
