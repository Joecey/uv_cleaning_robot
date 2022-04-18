"""omni_directional_base controller."""

#--------- Package imports ----- #

# import robot and keyboard packages (webots)
from controller import Robot, Keyboard

# import numpy as csv for occupancy grid manipulation
import numpy as np
import csv

print("Package import successful...")

#--------- World intialisation ----- #

# create the Robot instance.
TIME_STEP = 128
robot = Robot()

# setup keyboard for teleop
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

print("World intialisation succesful...")

#--------- Robot intialisation ----- #

# setup wheels
wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    wheels.append(robot.getDevice(name))
    
# print(wheels)
# print('wheels ready')

# max speed of wheels
max_speed = 3 #[rad/s]

# set initial position goal and speed
# make all wheels move same direction
for i in range(len(wheels)):    
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

print("Robot intialisation successful...")
    
#--------- Manual Movement Functions ----- #
def stop():
    # go forward at applied speed multiplier
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(0)

# forward
def forward(multiplier):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(multiplier * max_speed)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(multiplier * max_speed)
    wheels[3].setVelocity(0)


# backward
def backward(multiplier):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(-1 * multiplier * max_speed)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(-1 * multiplier * max_speed)
    wheels[3].setVelocity(0)

# right
def right(multiplier):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-1* multiplier * max_speed)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(-1 * multiplier * max_speed)

# left
def left(multiplier):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(0)
    wheels[1].setVelocity( multiplier * max_speed)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(multiplier * max_speed)

#--------- Occupancy grid import ----- #
# occupancy grid cell to x, y coordinates
def OG_to_XY(column = 0, row = 0,):
    # constants for cell width and height
    cell_width = 6/49
    cell_height = 4/35

    # max column = 49, max row = 35
    if column == 0:
        x = 0

    elif column > 49:
        x = (49 * cell_width) - (cell_width/2)

    else:
        x = (column * cell_width) - (cell_width/2)

    if row == 0:
        y = 0

    elif column > 35:
        y = (35 * cell_height) - (cell_height/2)

    else:   
        y = (row * cell_height) - (cell_height/2)

    return(x,y)

# open csv inside folder and export each row to list
with open('OG1.csv', 'rt') as f:
    reader = csv.reader(f)
    imported_og = list(reader)

# change all blanks to 0's
for i in range(len(imported_og)):
    for j in range(len(imported_og[i])):
        if imported_og[i][j] == "":
            imported_og[i][j] = "0"

        else:
            continue

# print each line of occupancy grid
# for i in range(len(imported_og)):
#     print(imported_og[i])

# -------- Start cleaning -----
print("Begin Cleaning!")

# FOR LOOP HERE
# speed multiplier variables from 0 to 1
# Make sure you click into 3d view before moving 
while robot.step(TIME_STEP) != -1:
    # get currently pressed key 
    key = keyboard.getKey()
    # print(key)
    # control robot using key preses
    if key == ord('W'):
        forward(1)
    
    elif key == ord('S'):
        backward(1)
        
    elif key == ord('A'):
        left(1)
        
    elif key == ord('D'):
        right(1)
    
    else:
        stop()