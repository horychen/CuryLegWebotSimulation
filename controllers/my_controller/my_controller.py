"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import GPS
import numpy as np
import math

AGENT = True
# AGENT = False

# create the Robot instance.
robot = Robot()
gps = GPS('gps')

hip = robot.getDevice('Hip_Joint_sensor')
motor_hip = robot.getDevice('screw_hip_joint')
screw_hip_joint_sensor = robot.getDevice('screw_hip_joint_sensor')

shank = robot.getDevice('shank_joint_sensor')
motor_shank = robot.getDevice('screw_shank_joint')
screw_shank_joint_sensor = robot.getDevice('screw_shank_joint_sensor')

timestep = int(robot.getBasicTimeStep())
print('time step:', timestep)

gps.enable(timestep)
motor_shank.enableForceFeedback(timestep)
motor_hip.enableForceFeedback(timestep)
screw_hip_joint_sensor.enable(timestep)
screw_shank_joint_sensor.enable(timestep)

# get the hip and shank joint sensors and enable them
hip.enable(timestep)
shank.enable(timestep)

# first line is hip, second line is shank
data = {
    "hip_joint_angle": [],
    "shank_joint_angle": [],
    "hip_motor_force": [],
    "shank_motor_force": [],
    "screw_hip_joint_sensor": [],
    "screw_shank_joint_sensor": []
}
save_flag = False

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

def record():
    data["hip_joint_angle"].append(hip.getValue())
    data["shank_joint_angle"].append(shank.getValue())
    data["hip_motor_force"].append(motor_hip.getForceFeedback())
    data["shank_motor_force"].append(motor_shank.getForceFeedback())
    data["screw_hip_joint_sensor"].append(screw_hip_joint_sensor.getValue())
    data["screw_shank_joint_sensor"].append(screw_shank_joint_sensor.getValue())

def save_record():
    with open('record.npy', 'wb') as f:
        np.save(f, data)
    print('Data saved')

define_trace()
motor_shank.setPosition(shank_trace[counter])
motor_hip.setPosition(hip_trace[counter])

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    if robot.getTime() < 2:
        record()
        continue
    jump()
    if robot.getTime() < 10:
        record()
    else:
        if not save_flag:
            save_flag = True
            save_record()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
