#!/usr/bin/env python3
import rospy

#import keyboard
from pynput import keyboard
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

letter = 'm'

#flags for key presses W A S D
w_pressed = False
a_pressed = False
s_pressed = False
d_pressed = False


def on_press(key):
    global letter
    global w_pressed    
    global a_pressed
    global s_pressed
    global d_pressed 
    letter = key.char
    # print(letter)
    #set flag to true
    if letter == 'w' or letter == 'W':
        w_pressed = True
    elif letter == 'a' or letter == 'A':
        a_pressed = True
    elif letter == 's' or letter == 'S':
        s_pressed = True
    elif letter == 'd' or letter == 'D':
        d_pressed = True



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
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=100)
    # Get the current time in seconds
    start = Time.now().to_sec()
    vel_msg = Twist()
    maxSpeed = 5
    maxAngularSpeed = 3.1 #radians/sec
    angularSpeed_r = 0.1 #radians/sec
    
    listener = keyboard.Listener(on_press=on_press , on_release= on_release)
    listener.start()

    print('use w a s d to control the robot')

    while not rospy.is_shutdown():
        # print('here')
        # print('lettter '+letter+' in while loopppp')
        #if keyboard.is_pressed('w') or keyboard.is_pressed('W') :
        # if letter =='w' or letter =='W' :
        if w_pressed:
            # ACCELERATE
            # print("enter acceleration")
            if vel_msg.linear.x < maxSpeed:
                vel_msg.linear.x += 0.01
        
        #if keyboard.is_pressed('s') or keyboard.is_pressed('S') :
        # if letter =='s' or letter =='S' :
        if s_pressed:
            # DECELERATE
            if vel_msg.linear.x > -maxSpeed:
                vel_msg.linear.x -= 0.01

        #if keyboard.is_pressed('a') or keyboard.is_pressed('A') :
        # if letter =='a' or letter =='A' :
        if a_pressed:
            # LEFT -- anticlockwise
            if vel_msg.angular.z < maxAngularSpeed:    
                vel_msg.angular.z += angularSpeed_r


        #if keyboard.is_pressed('d') or keyboard.is_pressed('D') :
        # if letter =='d' or letter =='D' :
        if d_pressed:
            # RIGHT -- clockwise
            if vel_msg.angular.z > -maxAngularSpeed:
                vel_msg.angular.z -= angularSpeed_r 


        #if not (keyboard.is_pressed('w') or keyboard.is_pressed('W')) and not (keyboard.is_pressed('s') or keyboard.is_pressed('S')):
        # if not(letter =='w' or letter =='W') and not(letter == 's' or letter == 'S') :
        if not w_pressed and not s_pressed:
            # DECELERATE WITH TIME PASSING 
            if vel_msg.linear.x > 0:
                vel_msg.linear.x -= 0.001
            elif vel_msg.linear.x < 0:
                vel_msg.linear.x += 0.001
            

        # if not(letter =='a' or letter =='A') and not(letter == 'd' or letter == 'D') :
        if not a_pressed and not d_pressed:
            vel_msg.angular.z = 0
            # DECELERATE WITH TIME PASSING
            if vel_msg.angular.z > 0:
                vel_msg.angular.z -= 0.001
            elif vel_msg.angular.z < 0:
                vel_msg.angular.z += 0.001


        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass