"""omni_directional_base controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# import keyboard package for teleop too
from controller import Robot, Keyboard

# create the Robot instance.
TIME_STEP = 128
robot = Robot()

# setup keyboard for teleop
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# setup gps
gps = robot.getGPS('gps')
gps.enable(TIME_STEP)

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

    # ----- GPS tracking -----
    gps_values = gps.getValues()

    msg = "GPS Values: "
    for each_val in gps_values:
        msg += " {0:0.5f}".format(each_val)

    print(msg)