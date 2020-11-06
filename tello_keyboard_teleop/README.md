# tello_keyboard_teleop
Generic Keyboard Teleop for the Tello drone in ROS

## Dependancies
This packages depends on the python libary pynput. You should install it before launch
```
pip install pynput
```

## Launch
Run for simultanious movement.
```
rosrun tello_keyboard_teleop tello_keyboard_teleop.py 
```

Run for simultanious movement.
```
rosrun tello_keyboard_teleop tello_keyboard_teleop_2.py 
```

With custom values.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.9 _turn:=0.8
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

## Output 
Twist message


