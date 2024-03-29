#!/usr/bin/env python3
import rospy
from robot_contro_pkg.msg import SensorSync
from prediction import prediction_stage
from correction import correction_stage

from utils import *
#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid

import globals as g

#publish new map data to map_topic
pub = rospy.Publisher('/map_topic',OccupancyGrid,queue_size = 5)

def mapping_callback(data):


    #print how much time each stage takes
        

    #predict robot position
    #x: x position of robot relative to world
    #y: y position of robot relative to world
    #theta: orientation of robot relative to world

    #time before prediction stage
    t1 = rospy.get_time()

    
    predictedState , predictionCovar = prediction_stage(data.twist.twist,data)
    #time after prediction stage

    # print("prediction stage time: ", (t2-t1))
    

    x = predictedState[0]
    y = -predictedState[1]

    # cov = data.pose.covariance

    # mean_corrected, cov_corrected = correction_EKF(x, y,theta,cov,xs,ys,data.ranges)
    theta = predictedState[2]


    x,y,theta = correction_stage(x,y,theta,data,predictionCovar,data.twist.twist)


    # print("correction stage time: ", (t2-t1))
    #get sensor data
    #x: x position of robot relative to world
    #y: y position of robot relative to world
    #xs: x positions of laser scans relative world
    #ys: y positions of laser scans relative world
    # x,y,xs,ys = get_sensor_data(data)


    data.pose.pose.position.x = x
    data.pose.pose.position.y = -y
    quaternion = quaternion_from_euler(0, 0, theta)
    data.pose.pose.orientation.x = quaternion[0]
    data.pose.pose.orientation.y = quaternion[1]
    data.pose.pose.orientation.z = quaternion[2]
    data.pose.pose.orientation.w = quaternion[3]
    _,_,xs,ys = get_sensor_data(data)#,x,y,theta)


    g.rx, g.ry = x, -y
    g.rtheta = theta
    #convert x,y to map coordinates
    #x1,y1 are the map coordinates of the robot
    x1,y1 = convert_to_map(x,y)
    #convert xs,ys to map coordinates
    #xs,ys are the map coordinates of the laser scans
    xs,ys = convert_to_map_array(xs,ys)

    #update map using sensor data and robot position
    update_map(x1,y1,xs,ys)  
    #time after map update
    t2 = rospy.get_time()

    print("map update time: ", (t2-t1))

    #publish map
    pub.publish(g.map)
    

    return



def main():
    rospy.init_node('mapping', anonymous=True)
    #initialize global variables
    g.init()
    g.prev_time = rospy.Time.now()
    #subscribe to sensor topic
    #buffer = size of SensorSync 

    #sync

    rospy.Subscriber("/sensors_topic", SensorSync, mapping_callback,queue_size=1,buff_size=2**24)
    #spin
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass