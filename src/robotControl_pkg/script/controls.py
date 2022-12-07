#!/usr/bin/env python3
import rospy

import keyboard
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def main():
    # Initialize Node with node name
    rospy.init_node('hello_controller')
    # Assign node as a publisher to this topic
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel
', Twist, queue_size=100)
    # Get the current time in seconds
    start = Time.now().to_sec()

    vel_msg = Twist()
    maxSpeed = 10
    angularSpeed_d = 5 #degrees/sec
    angularSpeed_r = (angularSpeed_d * 3.14) / 180 
    
    while not rospy.is_shutdown():

        if keyboard.is_pressed('w') or keyboard.is_pressed('W') :
            # ACCELERATE
            if vel_msg.linear.y < maxSpeed:
                vel_msg.linear.y += 0.5
        
        if keyboard.is_pressed('s') or keyboard.is_pressed('S') :
            # DECELERATE
            if vel_msg.linear.y > 0:
                vel_msg.linear.y -= 0.3

        if keyboard.is_pressed('a') or keyboard.is_pressed('A') :
            # LEFT -- anticlockwise
            vel_msg.angular.z += angularSpeed_r


        if keyboard.is_pressed('d') or keyboard.is_pressed('D') :
            # RIGHT -- clockwise
            vel_msg.angular.z -= angularSpeed_r


        if not (keyboard.is_pressed('w') or keyboard.is_pressed('W')) and not (keyboard.is_pressed('s') or keyboard.is_pressed('S')):
            # DECELERATE WITH TIME PASSING 
            if vel_msg.linear.y > 0:
                vel_msg.linear.y -= 0.1



        # If 5 seconds are passed
        if Time.now().to_sec() - start >= 5:
            msg = String()
            start = Time.now().to_sec()
            msg.data = 'Hello world at {}'.format(start)
            # Broadcast the message to the topic hello_sender
            pub.publish(msg)
            print('msg sent')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass