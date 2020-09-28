#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('tello_keyboard_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from pynput.keyboard import Key, Listener

msg = """
---------------------------------------------------------
Reading from the keyboard and publishing to Twist message
---------------------------------------------------------
Moving around, forward backward left right:
        ^    
   <         >
        v    

Key.up: 'forward',
Key.down: 'backward',
Key.left: 'left',
Key.right: 'right',


Rotating around z-axis and moving up down:
        w    
   a         d
        y    

'w': 'up',
'y': 'down',
'a': 'counter_clockwise',
'd': 'clockwise',

'+': increase max speeds by 0.1
'-': decrease max speeds by 0.1
default: 0.5

space: Takeof, tap again: land


ESC or CTRL to quit
---------------------------------------------------------
"""


class Commander:
    def __init__(self):
        # Set Parameters
        self.trans_speed = 0.5
        self.rot_speed = 0.5
        self.flying=False
        self.current_key = Key
        self.status = 0
        # Command publisher
        self.namespace = 'tello/'
        self.twist = Twist()
        self.pub_vel = rospy.Publisher(self.namespace+'cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher(self.namespace+'takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.namespace+'land', Empty, queue_size=1)

        # create two listeners
        # start keyboard listeners in other threads
        self.listener_left = Listener(on_press=self.on_press_left, on_release=self.on_release_left)
        self.listener_left.start()

        self.listener_right = Listener(on_press=self.on_press_right, on_release=self.on_release_right)
        self.listener_right.start()

        # Dictionaries
        self.arrowBindings = {
            Key.up: (1, 0, 0, 0),
            Key.down: (-1, 0, 0, 0),
            Key.left: (0, -1, 0, 0),
            Key.right: (0, 1, 0, 0),
        }
        self.charBindings = {
            'w': (0, 0, 1, 0),
            'y': (0, 0, -1, 0),
            'a': (0, 0, 0, -1),
            'd': (0, 0, 0, 1),
        }
        self.specialBindings = {
            Key.space: self.takeoff_land,
        }
        self.commandBindings = {
            '+': self.speed_up,
            '-': self.speed_down,
        }


    # Callback for keyboard listener for takeoff, yaw and height
    def on_press_left(self, key):
        self.current_key = key
        # print('{0} pressed'.format(self.current_key))
        try:
            if key in self.specialBindings.keys():
                self.specialBindings[key]()
            elif key.char in self.charBindings.keys():
                print('{0} pressed'.format(key))
                x = self.charBindings[key.char][0]
                y = self.charBindings[key.char][1]
                z = self.charBindings[key.char][2]
                th = self.charBindings[key.char][3]
                self.twist.linear.z = z * self.trans_speed
                self.twist.angular.z = th * self.rot_speed
                print("vel cmd x: %f  y: %f  z: %f  yaw: %f" % (self.twist.linear.y, self.twist.linear.x,
                    self.twist.linear.z, self.twist.angular.z))
                self.pub_vel.publish(self.twist)
            elif key.char in self.commandBindings.keys():
                self.commandBindings[key.char]()
                if (self.status == 14):
                    print(msg)
                self.status = (self.status + 1) % 15
        except Exception as e:
            print(e)

    # Callback for keyboard listener for x and y and speed adjustments
    def on_press_right(self, key):
        # print('{0} pressed'.format(key))
        self.current_key = key
        try:
            if key in self.arrowBindings.keys():
                print('{0} pressed'.format(self.current_key))
                x = self.arrowBindings[key][0]
                y = self.arrowBindings[key][1]
                z = self.arrowBindings[key][2]
                th = self.arrowBindings[key][3]
                # publish cmd
                self.twist.linear.y = x * self.trans_speed     # flip x and y to fit to tello
                self.twist.linear.x = y * self.trans_speed
                print('vel cmd x: %f  y: %f  z: %f  yaw: %f' % (self.twist.linear.y, self.twist.linear.x,
                    self.twist.linear.z, self.twist.angular.z))
                self.pub_vel.publish(self.twist)
            elif key.char in self.commandBindings.keys():
                self.commandBindings[key.char]()
                if (self.status == 14):
                    print(msg)
                self.status = (self.status + 1) % 15
        except Exception as e:
            print(e)


    # Callback for keyboard listener
    def on_release_left(self, key):
        try:
            if key.char in self.charBindings.keys():
                print("Reset to zero left")
                self.twist.linear.z = 0
                self.twist.angular.x = 0
                self.twist.angular.y = 0
                self.twist.angular.z = 0
                self.pub_vel.publish(self.twist)
        except Exception as e:
            print(e)

        if key == Key.esc or key == Key.ctrl:
            print("Stopping left...")
            return False

    # Callback for keyboard listener
    def on_release_right(self, key):
        if key in self.arrowBindings.keys():
            print("Reset to zero right")
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.pub_vel.publish(self.twist)
        if key == Key.esc or key == Key.ctrl:
            print("Stopping right...")
            return False

    def takeoff_land(self):
        empty = Empty()
        if not self.flying:
            print('Gooooooooooo')
            self.pub_takeoff.publish(empty)
            self.flying = True
        else:
            print('Landing')
            self.pub_land.publish(empty)
            self.flying = False

    def speed_up(self):
        if self.speed < 1.0:
            self.speed += 0.1
        print("Speed up, currently:\tspeed %s" % self.speed)

    def speed_down(self):
        if self.speed > 0.0:
            self.speed -= 0.1
        else:
            self.speed = 0.0
        print("Speed down, currently:\tspeed %s" % self.speed)


if __name__=="__main__":
    try:
        # Start ROS node
        rospy.init_node('tello_keyboard_teleop')
        print(msg)
        # create commander for controls
        node = Commander()
        rospy.spin()
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        node.pub_vel.publish(twist)
        print('Zero Command published')