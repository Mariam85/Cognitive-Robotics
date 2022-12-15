#!/usr/bin/env python3
import rospy
import message_filters
from rospy.rostime import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_contro_pkg.msg import SensorSync


global pub
# Assign node as a publisher to this topic
pub = rospy.Publisher('/sensors_topic',SensorSync,queue_size = 1)

def Odomlaser_callback(msg_f,msg_r,msg_o):
        #create custom message
        msg = SensorSync()
        #get time to be used in header for synchronization
        start = Time.now()
        #set header time stamp
        msg.header.stamp = start
        #set pose, twist, angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max
        msg.pose = msg_o.pose
        msg.twist = msg_o.twist
        msg.angle_min= msg_f.angle_min
        msg.angle_max= msg_r.angle_max
        #twice the angle increment because we are taking only 360 degree scan
        msg.angle_increment = msg_f.angle_increment * 2
        msg.time_increment = msg_f.time_increment * 2 
        msg.scan_time = msg_f.scan_time
        msg.range_min = msg_f.range_min
        msg.range_max = msg_f.range_max

        #take 360 degree scan from front and rear laser
        #take all the ranges from front laser
        msg.ranges[0:270]=msg_f.ranges [0:540:2]
        #take the remaining ranges from rear laser, from 180 to 360
        #discard the first 180 ranges from rear laser and the last 180
        msg.ranges[270:360]=msg_r.ranges[180:360:2]
        #take all the intensities from front laser
        msg.intensities = msg_f.intensities

        # Broadcast the message to the custom topic 
        pub.publish(msg)
        # print('msg sent')

def main():
    # Initialize Node with node name
    rospy.init_node('sensors')


    # Assign node as a subscriber to hello topic
    subFront = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)
    subRear = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
    subOdom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    
    # Synchronize the messages
    ts = message_filters.ApproximateTimeSynchronizer([subFront, subRear,subOdom],200,0.01,allow_headerless=False) #reduce 0.2
    ts.registerCallback(Odomlaser_callback)
    # Wait for messages
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass