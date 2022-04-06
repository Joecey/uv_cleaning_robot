"""omni_directional_base controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
TIME_STEP = 128
robot = Robot()

# setup wheels
wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    wheels.append(robot.getDevice(name))
    
# print(wheels)
# print('wheels ready')

# max speed of wheels
max_speed = 1.5 #[rad/s]

# set initial position goal and speed
# make all wheels move same direction
for i in range(len(wheels)):    
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
# MOVEMENT FUNCTIONS
# stop 
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


# FINISH THE REST OF THE FUNCTION HERE AND FIX COMMENTS

# FOR LOOP HERE
# speed multiplier variables from 0 to 1
left(1)