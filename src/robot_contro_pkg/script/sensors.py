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
        msg = SensorSync()
        start = Time.now()
        msg.header.stamp = start
        msg.pose = msg_o.pose
        msg.twist = msg_o.twist
        msg.angle_min= msg_f.angle_min
        msg.angle_max= msg_r.angle_max
        msg.angle_increment = msg_f.angle_increment
        msg.time_increment = msg_f.time_increment
        msg.scan_time = msg_f.scan_time
        msg.range_min = msg_f.range_min
        msg.range_max = msg_f.range_max

        #take 360 degree scan from front and rear laser
        msg.ranges[0:540]=msg_f.ranges
        msg.ranges[540:720]=msg_r.ranges[180:360]
        #msg.ranges [0:90]=msg_f.ranges[0:90]
        #msg.ranges[270:360]= msg_f.ranges[45:135]
        #msg.ranges[90:270]=msg_r.ranges[45:135]

        # msg.ranges[0:360]=msg_f.ranges[0:360]
        # msg.ranges[360:720]=msg_r.ranges[180:540]
        
        msg.intensities = msg_f.intensities

        # Broadcast the message to the topic hi_sender
        pub.publish(msg)
        print('msg sent')



def main():
    # Initialize Node with node name
    rospy.init_node('sensors')


    # Assign node as a subscriber to hello topic
    subFront = message_filters.Subscriber('/robot/front_laser/scan', LaserScan)
    subRear = message_filters.Subscriber('/robot/rear_laser/scan', LaserScan)
    subOdom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    
    ts = message_filters.ApproximateTimeSynchronizer([subFront, subRear,subOdom],200,0.2,allow_headerless=False)
    ts.registerCallback(Odomlaser_callback)
    # Wait for messages
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass