# import robot and motor pacakges from controller
from controller import Robot, Motor

# 64ms time step
TIME_STEP = 64

# max speed of 6.28m/s
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
# get the robot's left and right wheel 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

# no target for wheel positions
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# intiallly set robot's speed to 30% of max speed
leftMotor.setVelocity(0.3 * MAX_SPEED)
rightMotor.setVelocity(0.3 * MAX_SPEED)

while robot.step(TIME_STEP) != -1:
   pass