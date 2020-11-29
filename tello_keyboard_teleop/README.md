# tello_keyboard_teleop
Generic Keyboard Teleop for the Tello drone in ROS

## Dependancies
This packages depends on the python libary pynput. You should install it before launch
```
pip install pynput
```

## Launch
Run for sequential movement.
```
rosrun tello_keyboard_teleop tello_keyboard_teleop.py 
```

Run for simultanious movement.
```
rosrun tello_keyboard_teleop tello_keyboard_teleop_2.py 
```


## Usage
```
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

```

## Published Topics 
'tello/cmd_vel' as Twist  
'tello/takeoff' as Empty  
'tello/land' as Empty  
