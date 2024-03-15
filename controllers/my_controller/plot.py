''' Time step is 5ms '''
import matplotlib.pyplot as plt
import numpy as np
import os

SAMPLE_PERIOD = 0.005


def load_data():
    if not os.path.exists(os.path.dirname(__file__)+'/record.npy'):
        print(f'{os.path.dirname(__file__)}/record.npy No data file found')
        os._exit(0)
    with open(os.path.dirname(__file__)+'/record.npy', 'rb') as f:
        data = np.load(f, allow_pickle=True).item()
    return data

def plot_data(data):
    time = np.arange(0, len(data['hip_joint_angle']) * SAMPLE_PERIOD - 0.0001, SAMPLE_PERIOD)
    
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(time, data['hip_joint_angle'], '--o', markersize=0.5, linewidth=0.5)
    axs[0].set_title('Hip Joint Angle')
    axs[0].set_ylabel('Angle [rad]')
    
    axs[1].plot(time, data['shank_joint_angle'], '--o', markersize=0.5, linewidth=0.5)
    axs[1].set_title('Shank Joint Angle')
    axs[1].set_ylabel('Angle [rad]')
    plt.xlabel('Time [s]')
    plt.suptitle('Joint Angles over Time')
    
    
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(time, data['hip_torque'], '--o', markersize=0.5, linewidth=0.5)
    axs[0].set_title('Hip Torque')
    axs[0].set_ylabel('Torque [Nm]')
    
    axs[1].plot(time, data['shank_torque'], '--o', markersize=0.5, linewidth=0.5)
    axs[1].set_title('Shank Torque')
    axs[1].set_ylabel('Torque [Nm]')
    plt.xlabel('Time [s]')
    plt.suptitle('Torques over Time')
    
    
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(time[:-1], data['hip_screw_velocity'], '--o', markersize=0.5, linewidth=0.5)
    axs[0].set_title('Hip Velocity')
    axs[0].set_ylabel('Velocity [m/s]')
    
    axs[1].plot(time[:-1], data['shank_screw_velocity'], '--o', markersize=0.5, linewidth=0.5)
    axs[1].set_title('Shank Velocity')
    axs[1].set_ylabel('Velocity [Nm]')
    plt.xlabel('Time [s]')
    plt.suptitle('Screw Velocities over Time')
    
    fig, axs = plt.subplots(2, 1)
    axs[0].plot(time, data['hip_motor_force'], '--o', markersize=0.5, linewidth=0.5)
    axs[0].set_title('Hip Motor Force')
    axs[0].set_ylabel('Force [N]')
    
    axs[1].plot(time, data['shank_motor_force'], '--o', markersize=0.5, linewidth=0.5)
    axs[1].set_title('Shank Motor Force')
    axs[1].set_ylabel('Force [N]')
    plt.xlabel('Time [s]')
    plt.suptitle('Output Motor Forces over Time')
    
    
    plt.show()

def calc_torque(data):
    """Calc torque from motor force
    Formula: Torque  = (Force * Pitch) / (2 * pi * eta)
    where:  Pitch = 0.016, eta = 0.9
    """
    pitch = 0.016
    eta = 0.9
    hip_motor_force = data['hip_motor_force']
    shank_motor_force = data['shank_motor_force']
    data['hip_torque'] = [force * pitch / (2 * np.pi * eta) for force in hip_motor_force]
    data['shank_torque'] = [force * pitch / (2 * np.pi * eta) for force in shank_motor_force]
    return data

def calc_velocity(data):
    data['hip_screw_velocity'] = []
    data['shank_screw_velocity'] = []
    screw_hip_joint_sensor = data['screw_hip_joint_sensor']
    screw_shank_joint_sensor = data['screw_shank_joint_sensor']
    for i in range(1, len(data['hip_joint_angle'])):
        data['hip_screw_velocity'].append((screw_hip_joint_sensor[i] - screw_hip_joint_sensor[i-1]) / SAMPLE_PERIOD)
        data['shank_screw_velocity'].append((screw_shank_joint_sensor[i] - screw_shank_joint_sensor[i-1]) / SAMPLE_PERIOD)
    return data

if __name__ == "__main__":
    data = load_data()
    calc_torque(data)
    calc_velocity(data)
    plot_data(data)


