#!/usr/bin/env python3
import rospy

#import keyboard
from pynput import keyboard
from rospy.rostime import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

letter = 'm'

def on_press(key):
    global letter 
    letter = key.char
    print(letter)

def on_release(key):
    global letter
    letter = 'm'

def main():
    # Initialize Node with node name
    rospy.init_node('controls')
    # Assign node as a publisher to this topic
    pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=100)
    # Get the current time in seconds
    start = Time.now().to_sec()
    vel_msg = Twist()
    maxSpeed = 10
    angularSpeed_d = 1 #degrees/sec
    angularSpeed_r = (angularSpeed_d * 3.14) / 180 
    
    listener = keyboard.Listener(on_press=on_press , on_release= on_release)
    listener.start()

    while not rospy.is_shutdown():
        print('here')
        print('lettter '+letter+' in while loopppp')
        #if keyboard.is_pressed('w') or keyboard.is_pressed('W') :
        if letter =='w' or letter =='W' :
            # ACCELERATE
            print("enter acceleration")
            if vel_msg.linear.x < maxSpeed:
                vel_msg.linear.x += 0.1
        
        #if keyboard.is_pressed('s') or keyboard.is_pressed('S') :
        if letter =='s' or letter =='S' :
            # DECELERATE
            if vel_msg.linear.x > 0:
                vel_msg.linear.x -= 0.05

        #if keyboard.is_pressed('a') or keyboard.is_pressed('A') :
        if letter =='a' or letter =='A' :
           # LEFT -- anticlockwise
            vel_msg.angular.z += angularSpeed_r


        #if keyboard.is_pressed('d') or keyboard.is_pressed('D') :
        if letter =='d' or letter =='D' :
            print("enter right")

            # RIGHT -- clockwise
            vel_msg.angular.z -= angularSpeed_r


        #if not (keyboard.is_pressed('w') or keyboard.is_pressed('W')) and not (keyboard.is_pressed('s') or keyboard.is_pressed('S')):
        if not(letter =='w' or letter =='W') and not(letter == 's' or letter == 'S') :
            # DECELERATE WITH TIME PASSING 
            if vel_msg.linear.x > 0:
                vel_msg.linear.x -= 0.02



        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
