import rospy
import message_filters
from rospy.rostime import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_contro_pkg.msg import SensorSync
import numpy as np
from utils import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#import occupancy grid and occupancy grid metadata
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

def ThetaFromQuant(orientations):
    """
    Transform theta in radians from quanterion 
    """
    #
    xyzw = [ orientations.x, orientations.y, orientations.z, orientations.w]
    (roll, pitch, yaw) = euler_from_quaternion(xyzw)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # from 0 to 2pi
    return yaw

def ThetaFromOdom(odom):
    """"
    Get theta from Odometry
    """
    orientations = odom.pose.pose.orientation
    theta = ThetaFromQuant(orientations)
    return theta

#convert x and y to map coordinates
def convert_to_map(x,y):
    global metadata
    x = int(x/metadata.resolution)
    y = int(y/metadata.resolution)
    return x,y

def convert_to_map_array(xs,ys):
    global metadata
    xs = xs/metadata.resolution
    ys = ys/metadata.resolution
    xs = xs.astype(int)
    ys = ys.astype(int)
    return xs,ys

#convert map coordinates to x and y
def convert_to_xy(x,y):
    global metadata
    x = x*metadata.resolution
    y = y*metadata.resolution
    return x,y


        
#get x , y and theta from odometry
def get_pose(odom_data):
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    theta = ThetaFromOdom(odom_data)
    return x,y,theta

def get_laser_measuremts(laser_data):
    """
    Returns distances and thier crossponding thetas per laser scan
    """
    distances = np.array([])
    thetas = np.array([])
    for i in range(len(laser_data.ranges)):
        distance = laser_data.ranges[i]
        #discard above max or below min
        distance = laser_data.range_max if distance > laser_data.range_max else (laser_data.range_min if distance < laser_data.range_min else distance)
        distances = np.append(distances,distance)
        thetas = np.append(thetas,laser_data.angle_min + i*laser_data.angle_increment)
    return distances,thetas


def get_sensor_data(data):
    """
    Returns odometry and laser scan data
    """
    x,y,theta = get_pose(data)
    distances,thetas = get_laser_measuremts(data)
    return x,y,theta,distances,thetas

def get_sensor_data_as_xy(x,y,theta,distances,thetas):
    """
    Returns x and y of laser scan
    """
    #for each laser scan get x and y
    xs = np.array([])
    ys = np.array([])
    for i in range(len(distances)):
        x1 = x + distances[i]*np.cos(thetas[i]+theta)
        y1 = y + distances[i]*np.sin(thetas[i]+theta)
        xs = np.append(xs,x1)
        ys = np.append(ys,y1)
    return xs,ys

