#!/usr/bin/env python3
import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy.rostime import Time
import utils as util
import globals as g

#g.prev_time = Time.now()

def motion_model(state, u, dt):
    x, y, theta = state
    v, w = u
    if (w == 0):
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += 0
    else:
        x += -v/w * np.sin(theta) + v/w * np.sin(theta + w*dt)
        y += v/w * np.cos(theta) - v/w * np.cos(theta + w*dt)
        theta += w*dt
    return np.array([x, y, theta])


def jacob_motion(state, u, dt):
    x, y, theta = state
    v, w = u
    if (w == 0):
        G = np.array([[1, 0, -v*dt*np.sin(theta)],
                      [0, 1, v*dt*np.cos(theta)],
                      [0, 0, 1]])

        V = np.array([[dt*np.cos(theta), 0],
                       [dt*np.sin(theta), 0],
                       [0, 0]])
    else:
        G = np.array([[1, 0, v/w * (np.cos(theta + w*dt) - np.cos(theta))],
                    [0, 1, v/w * (np.sin(theta + w*dt) - np.sin(theta))],
                    [0, 0, 1]])
    
        V = np.array([[  (1/w)*(-np.sin(theta) + np.sin(theta + w*dt)), v/w**2 * (w*dt*np.cos(w*dt+theta)+np.sin(theta)-np.sin(w*dt+theta))],
                    [(1/w)*(np.cos(theta) - np.cos(theta + w*dt)), v/w**2 * (w*dt*np.sin(w*dt+theta)-np.cos(theta)+np.cos(w*dt+theta))],
                    [0, dt]])
    return G, V

def prediction(state, u, dt, covar, M):
    g = motion_model(state, u, dt)
    G, V = jacob_motion(g, u, dt)
    covar = np.dot(np.dot(G, covar), np.transpose(G)) + np.dot(np.dot(V, M), np.transpose(V))
    return g, covar

def prediction_stage(vel, odom):
    current_time = odom.header.stamp
    robot_pos = [odom.pose.pose.position.x, odom.pose.pose.position.y]
    theta = util.ThetaFromOdom(odom)
    robot_state = np.array([robot_pos[0], robot_pos[1], theta])
    robot_u = [vel.linear.x, vel.angular.z]
    dt = float((current_time - g.prev_time).to_sec())/10 #TODO: Check after correction step 
    #covar matrix, take x ,y,yaw  as 3x3 matrix from odom covariance
    covar = np.array([[odom.pose.covariance[0], odom.pose.covariance[1], odom.pose.covariance[5]],
                    [odom.pose.covariance[6], odom.pose.covariance[7], odom.pose.covariance[11]],
                    [odom.pose.covariance[30], odom.pose.covariance[31], odom.pose.covariance[35]]])
    M = np.array([[0.1, 0],
                    [0, 0.1]])
    robot_state, covar = prediction(robot_state, robot_u, dt, covar, M)
    g.prev_time = current_time
    return robot_state, covar
    


def main():
    rospy.init_node('prediction_stage', anonymous=True)
    subVel = message_filters.Subscriber('/robot/robotnik_base_control/cmd_vel', Twist)
    subOdom = message_filters.Subscriber('/robot/robotnik_base_control/odom', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([subVel, subOdom], 10, 0.1, allow_headerless=True)
    ts.registerCallback(prediction_stage)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
