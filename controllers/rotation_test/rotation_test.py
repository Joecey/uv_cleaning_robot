"""rotation_test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
max_speed = 6.28 #[rad/s]

# get the time step of the current world.
TIME_STEP = 128

# setup gps
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# setup pose gps
gps_pose = robot.getDevice('gps_pose')
gps_pose.enable(TIME_STEP)

wheels = []
wheel_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheel_names:
    wheels.append(robot.getDevice(name))


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

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    gps_values = gps.getValues()
    pose_values =  gps_pose.getValues()

    print("centre: " + str(gps_values[1]) + ",pose: " + str(pose_values[1]))


# Enter here exit cleanup code.
