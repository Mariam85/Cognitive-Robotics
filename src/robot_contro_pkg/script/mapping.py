#!/usr/bin/env python3
import rospy
from robot_contro_pkg.msg import SensorSync

from utils import *
#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid

import globals as g

#publish new map data to map_topic
pub = rospy.Publisher('/map_topic',OccupancyGrid,queue_size = 5)

def mapping_callback(data):
    print("mapping")
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

    #update map
    update_map(x1,y1,xs,ys)  

    #publish map
    pub.publish(g.map)
    

    return



def main():
    rospy.init_node('mapping', anonymous=True)
    #initialize global variables
    g.init()
    #subscribe to sensor topic
    #buffer = size of SensorSync 
    rospy.Subscriber("/sensors_topic", SensorSync, mapping_callback,queue_size=1,buff_size=2**24)
    #spin
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass