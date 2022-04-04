"""4_wheel_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# initialize motors and wheels
TIME_STEP = 64
robot = Robot()

wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    wheels.append(robot.getDevice(name))
    
# print(wheels)
# print('wheels ready')

# setup sensors
ds = []
ds_names = ['ds_right', 'ds_left']
for i in range(len(ds_names)):
    ds.append(robot.getDevice(ds_names[i]))
    
    # get readings every 64ms
    ds[i].enable(TIME_STEP)

# set max speed of robot
max_speed = 1.5 #[rad/s]

# make all wheels move same direction
for i in range(len(wheels)):    
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
# spin in place (counter clockwise)
# for i in range(len(wheels)):    
    # wheels[i].setPosition(float('inf'))
    
# wheels[0].setVelocity(-1 * speed)
# wheels[1].setVelocity(speed)
# wheels[2].setVelocity(-1 * speed)
# wheels[3].setVelocity(speed)

# while loop for constant moving
avoid_obs_counter = 0

while robot.step(TIME_STEP) != -1:
    # have robot move forward by default
    print(avoid_obs_counter)
    left_speed = 1.0
    right_speed = 1.0
    
    # if obstacle was seen
    if avoid_obs_counter > 0:
        # make robot spin until counter is set to 0
        avoid_obs_counter -= 1
        left_speed = 1.0
        right_speed = -1.0

    # if obstacle not seen, read sensors
    else:
    
        # if either sensor is too close, record obstacle
        for i in range(len(ds_names)):
            if ds[i].getValue() < 950.0:
            
                # this number is arbritary!
                # the bigger the number, the longer the robot rotates
                avoid_obs_counter = 80
                    
    wheels[0].setVelocity(left_speed)
    wheels[1].setVelocity(right_speed)
    wheels[2].setVelocity(left_speed)
    wheels[3].setVelocity(right_speed)