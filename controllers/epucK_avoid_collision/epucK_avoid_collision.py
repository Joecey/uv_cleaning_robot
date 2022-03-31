# import robot, distance sensor and motor packages for 
# object detection and motor driving 
from controller import Robot, DistanceSensor, Motor

# time step of 64ms
TIME_STEP = 64

# initialise robot
robot = Robot()

# initialize distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 
          'ps5', 'ps6', 'ps7']
      
# for each sensor, initialise    
for i in range(len(psNames)):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

# initial setup of robot motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# open feedback loop (continue simulation until exit command recieved)
while robot.step(TIME_STEP) != -1:
    # take in sensor data here
    # print('hi')
    psValues = []
    
    # for every sensor in ps...read their distance values
    for i in range(len(ps)):
        psValues.append(ps[i].getValue())
    
    # print distance values
    print(psValues)
    
    # based on what robot sees do this 
    #...