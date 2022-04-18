"""csv_reader controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

# check if numpy and opencv installed
import numpy as np
import csv
print("succesful")

# function to determine x and y coordinates for robot given OG cell 
# default cell value as origin 
def OG_to_XY(column = 0, row = 0,):
    # constants for cell width and height
    cell_width = 6/49
    cell_height = 4/35
    # print(cell_width, cell_height)

    # return x,y coordinate based on cell row and column given
    # return middle of cell (except if given 0)
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

# import csv from OG grid, and import each row 
# read csv file
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
for i in range(len(imported_og)):
    print(imported_og[i])

# apply x direction then y direction
x,y = OG_to_XY(55,35)
print(x,y)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
#     # Read the sensors:
#     # Enter here functions to read sensor data, like:
#     #  val = ds.getValue()

#     # Process sensor data here.

#     # Enter here functions to send actuator commands, like:
#     #  motor.setPosition(10.0)
#     pass

# Enter here exit cleanup code.
