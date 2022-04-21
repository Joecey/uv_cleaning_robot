"""omni_directional_base controller."""

#--------- Package imports ----- #

# import robot and keyboard packages (webots)
from controller import Robot, Keyboard

# import numpy as csv for occupancy grid manipulation
import numpy as np
import csv

# importmath for distance calcs
import math

print("Package import successful...")

#--------- World intialisation ----- #
# import csv of world
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

# set tolerance for occupancy grid and goals
cell_tolerance = 6
goal_tolerance = 4


# create the Robot instance.
TIME_STEP = 128
robot = Robot()

# setup keyboard for teleop
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# target goals
TL = [12,6] # 0 --> top left
BL = [12,39] # 1 --> top right
BR = [58, 39] # 2 --> bottom right
TR = [58, 6] # 3 --> top right

corner_goals = [TL, BL, BR, TR]

Mid_1 = [36,10]
Mid_2 = [39,29]

print("World intialisation succesful...")

#--------- Robot intialisation ----- #
# setup gps
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# setup pose gps
gps_pose = robot.getDevice('gps_pose')
gps_pose.enable(TIME_STEP)

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

#--------- re-orientate function ----- #
def re_orient():
    if robot.step(TIME_STEP) != -1:
        tolerance = 0.005
        base_value = gps.getValues()
        pose_value = gps_pose.getValues()
        base_x, base_y = base_value[0], base_value[1]
        pose_x, pose_y = pose_value[0], pose_value[1]
        print(base_x, pose_x)
        if base_x != pose_x:
            if pose_x < base_x:
                while abs(pose_x-base_x) > tolerance and robot.step(TIME_STEP) != -1:
                    base_value = gps.getValues()
                    pose_value = gps_pose.getValues()
                    base_x, base_y = base_value[0], base_value[1]
                    pose_x, pose_y = pose_value[0], pose_value[1]
                    cw_rotate(0.2)

            else:
                while abs(pose_x-base_x) > tolerance and robot.step(TIME_STEP) != -1:
                    base_value = gps.getValues()
                    pose_value = gps_pose.getValues()
                    base_x, base_y = base_value[0], base_value[1]
                    pose_x, pose_y = pose_value[0], pose_value[1]
                    ccw_rotate(0.2)

#--------- Manual Movement Functions ----- #
def cw_rotate(multiplier = 1):
    wheels[0].setVelocity(multiplier * max_speed)
    wheels[1].setVelocity(-1*multiplier * max_speed)
    wheels[2].setVelocity(-1* multiplier * max_speed)
    wheels[3].setVelocity( multiplier * max_speed)

def ccw_rotate(multiplier = 1):
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
def forward(multiplier = 1):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(multiplier * max_speed)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(multiplier * max_speed)
    wheels[3].setVelocity(0)


# backward
def backward(multiplier = 1):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(-1 * multiplier * max_speed)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(-1 * multiplier * max_speed)
    wheels[3].setVelocity(0)

# right
def right(multiplier = 1):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(-1* multiplier * max_speed)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(-1 * multiplier * max_speed)

# left
def left(multiplier = 1):
    # go forward at applied speed multiplier
    wheels[0].setVelocity(0)
    wheels[1].setVelocity( multiplier * max_speed)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(multiplier * max_speed)

# -------- cleaning point protocals --------
def TL_start():
    re_orient()
    stop()
    TL_xy = OG_to_XY(TL[0], TL[1])
    y_reach = False
    x_reach = False

    # go to correct y position
    while y_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)
        
        # keep moving until obstacle is seen
        try: 
            if imported_og[current_row - goal_tolerance][current_column] == "1":
                y_reach = True

            else:
                forward(1)
        # if out of index, then we are already close to the walls
        except IndexError:
            y_reach = True

    # stop wheels briefly to prevent slipping
    stop()

    # go to correct x position
    while x_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        # keep moving until obstacle is seen
        try:
            if imported_og[current_row][current_column - goal_tolerance] == "1":
                x_reach = True
            else:
                left(1)
        except IndexError:
            x_reach = True
        

    stop()
    # begin cleaning protocal from TL to BL
    cleaning = True
    while cleaning is True and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        if abs(BL[1] - current_row) <= goal_tolerance:
            cleaning = False
        else:
            # move right until clear
            if imported_og[current_row + cell_tolerance][current_column] == "1":
                
                # get current position and create a temp goal where obstacle is cleared
                current_pos = gps.getValues()
                current_x, current_y = current_pos[0], abs(current_pos[1])
                current_column, current_row = XY_to_OG(current_x, current_y)
                cleared_column = current_column + cell_tolerance

                avoided = False

                # keep moving right until temp goal is reached
                while avoided == False and robot.step(TIME_STEP) != -1:
                    current_pos = gps.getValues()
                    current_x, current_y = current_pos[0], abs(current_pos[1])
                    current_column, current_row = XY_to_OG(current_x, current_y)

                    if cleared_column == current_column:
                        avoided = True
                    else:
                        right(1)

            # otherwise, if there is no obstacle
            else: 
                backward(0.8)
    stop()            
            
def BL_start():
    re_orient()
    stop()
    BL_xy = OG_to_XY(BL[0], BL[1])
    y_reach = False
    x_reach = False

    # go to correct y position
    while y_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        try:
            if imported_og[current_row + goal_tolerance][current_column] == "1":
                y_reach = True

            else:
                backward(1)
        except IndexError:
            y_reach = True

    # stop wheels briefly to prevent slipping
    stop()

    # go to correct x position
    while x_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        try:
            if imported_og[current_row][current_column - goal_tolerance] == "1":
                x_reach = True

            else:
                left(1)

        except IndexError:
            x_reach = True

    # begin cleaning protocal from BL to BR
    stop()
    cleaning = True
    while cleaning is True and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        if abs(BR[0] - current_column) <= goal_tolerance:
            cleaning = False
        else:
            # move up until clear
            if imported_og[current_row][current_column + cell_tolerance] == "1":
                
                # get current position and create a temp goal where obstacle is cleared
                current_pos = gps.getValues()
                current_x, current_y = current_pos[0], abs(current_pos[1])
                current_column, current_row = XY_to_OG(current_x, current_y)
                cleared_row = current_row + cell_tolerance

                avoided = False

                # keep moving forward  until temp goal is reached
                while avoided == False and robot.step(TIME_STEP) != -1:
                    current_pos = gps.getValues()
                    current_x, current_y = current_pos[0], abs(current_pos[1])
                    current_column, current_row = XY_to_OG(current_x, current_y)

                    if cleared_row == current_row:
                        avoided = True
                    else:
                        forward()

            # otherwise, if there is no obstacle
            else: 
                right(0.8)
    stop()  

def BR_start():
    re_orient()
    stop()
    BR_xy = OG_to_XY(BR[0], BR[1])
    y_reach = False
    x_reach = False

    # go to correct y position
    while y_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        try:
            if imported_og[current_row + goal_tolerance][current_column] == "1":
                y_reach = True

            else:
                backward(1)
        except IndexError:
            y_reach = True

    # stop wheels briefly to prevent slipping
    stop()

    # go to correct x position
    while x_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        try:
            if imported_og[current_row][current_column + goal_tolerance] == "1":
                x_reach = True

            else:
                right(1)
        # if index is out of range, assume that we are already at the walls
        except IndexError:
            x_reach = True

    # begin cleaning protocal from BR to TR
    cleaning = True
    while cleaning is True and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        if abs(TR[1] - current_row) <= goal_tolerance:
            cleaning = False
        else:
            # move right until clear
            if imported_og[current_row - cell_tolerance][current_column] == "1":
                
                # get current position and create a temp goal where obstacle is cleared
                current_pos = gps.getValues()
                current_x, current_y = current_pos[0], abs(current_pos[1])
                current_column, current_row = XY_to_OG(current_x, current_y)
                cleared_column = current_column + cell_tolerance

                avoided = False

                # keep moving left until temp goal is reached
                while avoided == False and robot.step(TIME_STEP) != -1:
                    current_pos = gps.getValues()
                    current_x, current_y = current_pos[0], abs(current_pos[1])
                    current_column, current_row = XY_to_OG(current_x, current_y)

                    if cleared_column == current_column:
                        avoided = True
                    else:
                        left(1)

            # otherwise, if there is no obstacle
            else: 
                forward(0.8)
    stop()

def TR_start():
    re_orient()
    stop()
    TR_xy = OG_to_XY(TR[0], TR[1])
    y_reach = False
    x_reach = False

    # go to correct y position
    while y_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)


        try:
            if imported_og[current_row - goal_tolerance][current_column] == "1":
                y_reach = True

            else:
                forward(1)
        except IndexError:
            y_reach = True

    # stop wheels briefly to prevent slipping
    stop()

    # go to correct x position
    while x_reach is False and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        try:
            if imported_og[current_row][current_column + goal_tolerance] == "1":
                x_reach = True
            else:
                right(1)
        except IndexError:
            x_reach = True

    stop()
    # begin cleaning protocal from TR to TL
    stop()
    cleaning = True
    while cleaning is True and robot.step(TIME_STEP) != -1:
        current_pos = gps.getValues()
        current_x, current_y = current_pos[0], abs(current_pos[1])
        current_column, current_row = XY_to_OG(current_x, current_y)

        if abs(TL[0] - current_column) <= goal_tolerance:
            cleaning = False
        else:
            # move up until clear
            if imported_og[current_row][current_column - cell_tolerance] == "1":
                
                # get current position and create a temp goal where obstacle is cleared
                current_pos = gps.getValues()
                current_x, current_y = current_pos[0], abs(current_pos[1])
                current_column, current_row = XY_to_OG(current_x, current_y)
                cleared_row = current_row + cell_tolerance

                avoided = False

                # keep moving backward until temp goal is reached
                while avoided == False and robot.step(TIME_STEP) != -1:
                    current_pos = gps.getValues()
                    current_x, current_y = current_pos[0], abs(current_pos[1])
                    current_column, current_row = XY_to_OG(current_x, current_y)

                    if cleared_row == current_row:
                        avoided = True
                    else:
                        backward()

            # otherwise, if there is no obstacle
            else: 
                left(0.8)
    stop()  

#--------- Occupancy grid manipulation ----- #
# x y coords to occupancy grid column and row 
def XY_to_OG(x = 0, y = 0):
    max_column = 62
    max_row = 42

    cell_width = 6/max_column
    cell_height = 4/max_row

    y_abs = abs(y)

    column = math.floor(x/cell_width)
    row = math.floor(y_abs/cell_height)

    return(column, row)


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


# print each line of occupancy grid
# for i in range(len(imported_og)):
#     print(imported_og[i])

# -------- Start cleaning -----
print("Begin Cleaning!")

# Find nearest goal
# print(corner_goals)

# print(start_x, start_y)
if robot.step(TIME_STEP) != -1:
    # get start position of robot
    start_pos = gps.getValues()
    start_x, start_y = start_pos[0], abs(start_pos[1])

    # found_nearest = False
    shortest_d = 9999
    corner_point = 0

    for check in range(len(corner_goals)):
        test = corner_goals[check]
        xy_coords = OG_to_XY(test[0], test[1])
        # calculate distance
        temp1 = (xy_coords[0] - start_x) ** 2
        temp2 = (xy_coords[1] - start_y) ** 2

        dist = math.sqrt(temp1 + temp2)
        # print(dist)
        # if dist less than shortest value, replace
        if dist < shortest_d:
            shortest_d = dist
            corner_point = check

    # found_nearest = True
    print(shortest_d, corner_point)

    # ------ begin automated cleaning -----
    if corner_point == 0:
        TL_start()
        BL_start()
        BR_start()
        TR_start()

    elif corner_point == 1:
        BL_start()
        BR_start()
        TR_start()
        TL_start()

    elif corner_point == 2:
        BR_start()
        TR_start()
        TL_start()
        BL_start()

    elif corner_point == 3:
        TR_start()
        TL_start()
        BL_start()
        BR_start()

# -------- Begin manual control -----
print("Manual control active")
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