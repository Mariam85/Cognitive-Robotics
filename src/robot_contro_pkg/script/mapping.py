#!/usr/bin/env python3
import rospy
import message_filters
from rospy.rostime import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_control_pkg.msg import SensorSync

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data.pose.pose.position.x)

def main():
    rospy.init_node('mapping', anonymous=True)
    rospy.Subscriber("/sensors_topic", SensorSync, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass