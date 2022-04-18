"""csv_reader controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

# check if numpy and opencv installed
import numpy as np
import csv
print("succesful")

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
