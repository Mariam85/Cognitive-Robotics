
import numpy as np
#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

#import odometry
from nav_msgs.msg import Odometry


def init():
    # define the global variables
    global metadata
    origin =  Odometry()
    origin.pose.pose.position.x = -int(100*0.1)
    origin.pose.pose.position.y = -int(100*0.1)
    origin.pose.pose.position.z = 0
    origin.pose.pose.orientation.x = 0
    origin.pose.pose.orientation.y = 0
    origin.pose.pose.orientation.z = 0
    origin.pose.pose.orientation.w = 0

    metadata = MapMetaData(resolution=0.1, width=200, height=200, origin=origin.pose.pose)
    global map
    map = OccupancyGrid(info=metadata,data=np.full((metadata.width*metadata.height),-1.0))
    map.header.frame_id = "robot_map"
    global map_data
    #2d array of the map
    map_data = np.full((metadata.height,metadata.width),0.0)
    #threshold for occupancy
    global occupied_threshold
    global free_threshold
    occupied_threshold = 0.7
    free_threshold = 0.3

