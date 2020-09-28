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
        self.speed = 0.5
        self.flying=False
        self.current_key = Key
        self.status = 0
        # Start ROS node
        rospy.init_node('tello_keyboard_teleop')
        # Command publisher
        self.namespace = 'tello/'
        self.pub_vel = rospy.Publisher(self.namespace+'cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher(self.namespace+'takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.namespace+'land', Empty, queue_size=1)

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
        # initialize keyboard listener
        # Collect events until released
        # with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
        #    listener.join()

    # Callback for keyboard listener
    def on_press(self, key):
        # print('{0} pressed'.format(key))
        self.current_key = key
        print('{0} pressed'.format(self.current_key))
        try:
            if key in self.arrowBindings.keys():
                x = self.arrowBindings[key][0]
                y = self.arrowBindings[key][1]
                z = self.arrowBindings[key][2]
                th = self.arrowBindings[key][3]
                print('velocity command direction x: ', x, ' | direction y: ', y)
                # publish cmd
                twist = Twist()
                twist.linear.y = x * node.speed     # flip x and y to fit to tello
                twist.linear.x = y * node.speed
                twist.linear.z = z * node.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.pub_vel.publish(twist)
            elif key in self.specialBindings.keys():
                self.specialBindings[key]()
            elif key.char in self.charBindings.keys():
                x = self.charBindings[key.char][0]
                y = self.charBindings[key.char][1]
                z = self.charBindings[key.char][2]
                th = self.charBindings[key.char][3]
                print('velocity command direction z: ', z, ' | around z: ', th)
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = z * node.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = th * node.speed
                self.pub_vel.publish(twist)
            elif key.char in self.commandBindings.keys():
                self.commandBindings[key.char]()
                if (self.status == 14):
                    print(msg)
                self.status = (self.status + 1) % 15
        except Exception as e:
            print(e)

    # Callback for keyboard listener
    def on_release(self, key):
        print("Reset to zero")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_vel.publish(twist)
        if key == Key.esc or key == Key.ctrl:
            print("Stopping...")
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
    # create commander for controls
    node = Commander()

    try:
        print(msg)
        # start keyboard listener in other thread
        with Listener(
                on_press=node.on_press, on_release=node.on_release) as listener:
            listener.join()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        node.pub_vel.publish(twist)
        print('Zero Command published')