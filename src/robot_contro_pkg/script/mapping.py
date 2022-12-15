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

    #get sensor data
    #x: x position of robot relative to world
    #y: y position of robot relative to world
    #xs: x positions of laser scans relative world
    #ys: y positions of laser scans relative world
    x,y,xs,ys = get_sensor_data(data)

    #convert x,y to map coordinates
    #x1,y1 are the map coordinates of the robot
    x1,y1 = convert_to_map(x,y)
    #convert xs,ys to map coordinates
    #xs,ys are the map coordinates of the laser scans
    xs,ys = convert_to_map_array(xs,ys)

    #update map using sensor data and robot position
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