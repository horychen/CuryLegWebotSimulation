from matplotlib import pyplot as plt
import numpy as np


with open("linear.txt", "r") as f:
    data = f.readlines()
    data = [list(map(float, line.strip().split())) for line in data]
    data = np.array(data)

plt.plot(data[:, 1], label="linear theta 1")
plt.plot(data[:, 2], label="linear theta 2")

plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.show()

with open("sinusoidal.txt", "r") as f:
    data = f.readlines()
    data = [list(map(float, line.strip().split())) for line in data]
    data = np.array(data)

plt.plot(data[:, 1], label="sinusoidal theta 1")
plt.plot(data[:, 2], label="sinusoidal theta 2")

plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.show()

with open("bezier.txt", "r") as f:
    data = f.readlines()
    data = [list(map(float, line.strip().split())) for line in data]
    data = np.array(data)

plt.plot(data[:, 1], label="bezier theta 1")
plt.plot(data[:, 2], label="bezier theta 2")

plt.xlabel("time [s]")
plt.ylabel("angle [rad]")
plt.legend()
plt.show()