
import numpy as np
#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

#import odometry
from nav_msgs.msg import Odometry

#use this in main to initialize global variables
def init():
    #define the map size and resolution
    width , height , resolution = 1000,1000,0.08
    #create map metadata with preferred resolution and size
    global metadata
    origin =  Odometry()
    #set origin to the center of the map
    origin.pose.pose.position.x = -int((width/2)*resolution)
    origin.pose.pose.position.y = -int((height/2)*resolution)
    #set orientation to 0
    origin.pose.pose.position.z = 0
    origin.pose.pose.orientation.x = 0
    origin.pose.pose.orientation.y = 0
    origin.pose.pose.orientation.z = 0
    origin.pose.pose.orientation.w = 0

    #create map metadata
    metadata = MapMetaData(resolution=resolution, width=width, height=height, origin=origin.pose.pose)
    global map
    #create map with -1 values, and prevously defined metadata
    map = OccupancyGrid(info=metadata,data=np.full((metadata.width*metadata.height),-1.0).flatten().astype(np.int8))
    #set map frame id to robot_map, allows RVIZ to transform to robot frame
    map.header.frame_id = "robot_map"
    global map_data
    #2d array of the map, holds log odds values, initialized to 0.0 (0.5 or P == 0.5)
    map_data = np.full((metadata.height,metadata.width),0.0)
    #threshold for occupancy
    global occupied_threshold
    global free_threshold
    occupied_threshold = 0.95
    free_threshold = 0.45
    #log odds values for occupied and free
    global log_odds_occ
    global log_odds_free
    log_odds_occ = np.log(occupied_threshold/(1-occupied_threshold))
    log_odds_free = np.log(free_threshold/(1-free_threshold))
    #prediction stage variables
    global prev_time

    global distancesCorrection
    distancesCorrection = [-1,-1]
    global thetasCorrection
    thetasCorrection = [-1,-1]

    global rx
    global ry
    global rtheta
    rx = 0.0
    ry = 0.0
    rtheta = 0.0

    global xsPredicted
    global ysPredicted
    #xs predicted is 360 long
    # ys predicted is 360 long
    xsPredicted = np.zeros(360)
    ysPredicted = np.zeros(360)
    global predicted_measurements
    predicted_measurements = np.full((360),-1.0) 