#!/usr/bin/env python3
import rospy
import message_filters
from rospy.rostime import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_contro_pkg.msg import SensorSync
import numpy as np
from utils import *

#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

#publish new map data to map_topic
pub = rospy.Publisher('/map_topic',OccupancyGrid,queue_size = 100)


metadata = MapMetaData()
metadata.resolution = 0.05
metadata.width = 100
metadata.height = 100

map = OccupancyGrid()
map.info = metadata
#set all cells to 0.5 prior of numpy array of width*height 
map.data = np.full((metadata.width*metadata.height),0.5)

def mapping_callback(data):
    #access global map
    global map
    
    #get sensor data
    #x: x position of robot
    #y: y position of robot
    #theta: theta of robot (yaw)
    #distances: distances of laser scans
    #angles: angles of laser scans
    x,y,theta,distances,angles = get_sensor_data(data)

    #instead of distances per laser scan, get the x and y coordinates of the laser hits
    xs,ys = get_sensor_data_as_xy(x,y,theta,distances,angles)
    
    #convert x and y to map coordinates (discretization)
    #x1 and y1 are the map coordinates of the robot
    x1,y1 = convert_to_map(x,y)
    #xs and ys are the map coordinates of the laser scans
    xs,ys = convert_to_map_array(xs,ys)

    #loop through all laser hits and set the corresponding cell probabilities

    
  


    
    
    

  
    

    



def main():
    rospy.init_node('mapping', anonymous=True)
    rospy.Subscriber("/sensors_topic", SensorSync, mapping_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass