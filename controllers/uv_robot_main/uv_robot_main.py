"""omni_directional_base controller."""

#--------- Package imports ----- #

# import robot and keyboard packages (webots)
from controller import Robot, Keyboard

# import numpy as csv for occupancy grid manipulation
import numpy as np
import csv

# import math for distance calcs
import math

print("Package import successful...")

#--------- World intialisation ----- #

# create the Robot instance.
TIME_STEP = 128
robot = Robot()

# setup keyboard for teleop
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# target goals
TL = [12,6] # 0
BL = [12,37] # 1
BR = [56, 37] # 2
TR = [56, 6] # 3

corner_goals = [TL, BL, BR, TR]

Mid_1 = [36,10]
Mid_2 = [39,29]

print("World intialisation succesful...")

#--------- Robot intialisation ----- #
# setup gps
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# setup wheels
wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    wheels.append(robot.getDevice(name))
    
# print(wheels)
# print('wheels ready')

# max speed of wheels
max_speed = 6.28 #[rad/s]

# set initial position goal and speed
# make all wheels move same direction
for i in range(len(wheels)):    
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

print("Robot intialisation successful...")
    
#--------- Manual Movement Functions ----- #
def cw_rotate(multiplier):
    wheels[0].setVelocity(multiplier * max_speed)
    wheels[1].setVelocity(-1*multiplier * max_speed)
    wheels[2].setVelocity(-1* multiplier * max_speed)
    wheels[3].setVelocity( multiplier * max_speed)

def ccw_rotate(multiplier):
    wheels[0].setVelocity(-1 *multiplier * max_speed)
    wheels[1].setVelocity(multiplier * max_speed)
    wheels[2].setVelocity(multiplier * max_speed)
    wheels[3].setVelocity(-1 *multiplier * max_speed)

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
def OG_to_XY(column = 0, row = 0):
    # return x,y coordinate based on cell row and column given
    # return middle of cell (except if given 0)
    max_column = 62
    max_row = 42

     # constants for cell width and height
    cell_width = 6/max_column
    cell_height = 4/max_row

    if column == 0:
        x = 0

    elif column > max_column:
        x = (max_column * cell_width) - (cell_width / 2)

    else:
        x = (column * cell_width) - (cell_width / 2)

    if row == 0:
        y = 0

    elif row > max_row:
        y = (max_row * cell_height) - (cell_height / 2)

    else:
        y = (row * cell_height) - (cell_height / 2)

    return(x,y)

# open csv inside folder and export each row to list
with open('SampleTestMap1.csv', 'rt') as f:
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

found_nearest = False
shortest_d = 9999
corner_point = 0

while robot.step(TIME_STEP) != -1:

    # ------ begin automated cleaning -----
    while found_nearest == False:
        # Find nearest goal
        # print(corner_goals)

        # get start position of robot
        start_pos = gps.getValues()
        start_x, start_y = start_pos[0], abs(start_pos[1])

        # print(start_x, start_y)

        for check in range(len(corner_goals)):
            test = corner_goals[check]
            xy_coords = OG_to_XY(test[0], test[1])
            # calculate distance
            temp1 = (xy_coords[0] - start_x)**2
            temp2 = (xy_coords[1] - start_y)**2

            dist = math.sqrt(temp1 + temp2)
            # print(dist)
            # if dist less than shortest value, replace
            if dist < shortest_d:
                shortest_d = dist
                corner_point = check

        found_nearest = True
        print(shortest_d, corner_point)
        print("Manual control active")

    # -------- Begin manual control -----
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

    elif key == ord('Q'):
        ccw_rotate(0.5)

    elif key == ord('E'):
        cw_rotate(0.5)

    else:
        stop()

    # ----- GPS tracking -----
    gps_values = gps.getValues()

    msg = "GPS Values: "
    for each_val in gps_values:
        msg += " {0:0.5f}".format(each_val)

    # print(msg)