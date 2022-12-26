#!/usr/bin/env python3
import rospy

#import keyboard
from pynput import keyboard
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

letter = 'm'

#flags for key presses W A S D
#when a flag is true, the robot will move in that direction
w_pressed = False
a_pressed = False
s_pressed = False
d_pressed = False


#when a key is pressed, set the flag to true
def on_press(key):
    global letter
    global w_pressed    
    global a_pressed
    global s_pressed
    global d_pressed 
    letter = key.char
    
    #set flag to true
    if letter == 'w' or letter == 'W':
        w_pressed = True
    elif letter == 'a' or letter == 'A':
        a_pressed = True
    elif letter == 's' or letter == 'S':
        s_pressed = True
    elif letter == 'd' or letter == 'D':
        d_pressed = True


#when a key is released, set the flag to false
def on_release(key):
    global letter
    global w_pressed
    global a_pressed
    global s_pressed
    global d_pressed
    #set flag to false
    if key.char == 'w' or key.char == 'W':
        w_pressed = False
    elif key.char == 'a' or key.char == 'A':
        a_pressed = False
    elif key.char == 's' or key.char == 'S':
        s_pressed = False
    elif key.char == 'd' or key.char == 'D':
        d_pressed = False
    letter = 'm'


def main():
    # Initialize Node with node name
    rospy.init_node('controls')
    # Assign node as a publisher to this topic
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=1)
    # Get the current time in seconds
    start = Time.now().to_sec()
    vel_msg = Twist()
    maxSpeed = 1
    maxAngularSpeed = 1 #radians/sec
    angularSpeed_r = 0.1 #radians/sec
    
    # Create a listener for keyboard input
    listener = keyboard.Listener(on_press=on_press , on_release= on_release)
    #start the thread
    listener.start()

    print('\n\n\n\n')
    print('use w a s d to control the robot\n\n')


    while not rospy.is_shutdown():

        # ACCELERATE
        if w_pressed:
            if vel_msg.linear.x < maxSpeed:
                vel_msg.linear.x += 0.01
        
        # DECELERATE
        if s_pressed:
            if vel_msg.linear.x > -maxSpeed:
                vel_msg.linear.x -= 0.01

        # LEFT -- anticlockwise
        if a_pressed:
            if vel_msg.angular.z < maxAngularSpeed:    
                vel_msg.angular.z += angularSpeed_r


        # RIGHT -- clockwise
        if d_pressed:
            if vel_msg.angular.z > -maxAngularSpeed:
                vel_msg.angular.z -= angularSpeed_r 



        # DECELERATE velocity WITH TIME PASSING 
        if not w_pressed and not s_pressed:
            if vel_msg.linear.x > 0:
                vel_msg.linear.x -= 0.001
            elif vel_msg.linear.x < 0:
                vel_msg.linear.x += 0.001

            if vel_msg.linear.x < 0.01 and vel_msg.linear.x > -0.01:
                vel_msg.linear.x = 0
            

        # DECELERATE angular vel WITH TIME PASSING
        if not a_pressed and not d_pressed:
            vel_msg.angular.z = 0
            if vel_msg.angular.z > 0:
                vel_msg.angular.z -= 0.001
            elif vel_msg.angular.z < 0:
                vel_msg.angular.z += 0.001

            if vel_msg.angular.z < 0.01 and vel_msg.angular.z > -0.01:
                vel_msg.angular.z = 0
                

        #publish the custom message to the topic
        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass