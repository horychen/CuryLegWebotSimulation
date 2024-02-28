"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import GPS
import math

AGENT = True
# AGENT = False

# create the Robot instance.
robot = Robot()
gps = GPS('gps')
motor_shank = robot.getDevice('screw_shank_joint')
motor_hip = robot.getDevice('screw_hip_joint')

timestep = int(robot.getBasicTimeStep())

gps.enable(timestep)
motor_shank.enableTorqueFeedback(timestep)
motor_hip.enableTorqueFeedback(timestep)

shank_limit = [-0.02, 0.08]
hip_limit = [-0.009, 0.06]
shank_trace, hip_trace = [], []
period = [100, 4]
T = sum(period)


counter = 0

def nonlinear_trace(total, start_val, end_val):
    mapped_step = math.pi / (total - 1)
    result = []
    for i in range(total):
        step = start_val + (end_val - start_val) * (math.sin(mapped_step * i - math.pi / 2) + 1) / 2
        result.append(step)
    return result

def linear_step(total, start_val, end_val):
    result = []
    for i in range(total):
        step = start_val + i * ((end_val - start_val) / (total - 1))
        result.append(step)
    return result

def define_trace():
    global shank_trace, hip_trace
    shank_trace += nonlinear_trace(period[0], shank_limit[0], shank_limit[1])
    hip_trace += nonlinear_trace(period[0], hip_limit[1], hip_limit[0])
    shank_trace += nonlinear_trace(period[1], shank_limit[1], shank_limit[0])
    hip_trace += nonlinear_trace(period[1], hip_limit[0], hip_limit[1])

def jump():
    global counter
    if AGENT:
        counter += 1
        counter %= T
        motor_shank.setPosition(shank_trace[counter])
        motor_hip.setPosition(hip_trace[counter])
    # print(f"{gps.getValues()[2]=:.4f}", end='\t')
    # print(f"{motor_shank.getForceFeedback()=:.1f}", end='\t')
    # print(f"{motor_hip.getForceFeedback()=:.1f}")

define_trace()
motor_shank.setPosition(shank_trace[counter])
motor_hip.setPosition(hip_trace[counter])

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if robot.getTime() < 2:
        continue
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    jump()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
