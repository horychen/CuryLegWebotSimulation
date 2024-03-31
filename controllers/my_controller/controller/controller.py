import numpy as np
from matplotlib import pyplot as plt

class CuryController:
    def __init__(self, **kwargs):
        self.L1 = 0.4
        self.L2 = 0.39495
        self.theta1, self.theta2 = 0.0, 0.0
        self.T = 1.0
        self.height_limit = [0.2, 0.7]
        self.c = np.array([[0.0, 0.0], [0.1, 1.0], [0.21, 0.93], [1.0, 1.0]])
        self.order = len(self.c)

    def calc_theta_from_height(self, height):
        self.theta1 = np.arccos((self.L1**2 + height**2 - self.L2**2) / (2*self.L1*height))
        self.theta2 = np.arccos((self.L2**2 + height**2 - self.L1**2) / (2*self.L2*height))

    def linear(self, t):
        period = t // self.T
        t = t % self.T
        if period % 2 == 0:
            height = self.height_limit[0] + (self.height_limit[1] - self.height_limit[0]) * t / self.T
        else:
            height = self.height_limit[1] - (self.height_limit[1] - self.height_limit[0]) * t / self.T
        self.calc_theta_from_height(height)
        return self.theta1, self.theta1 + self.theta2

    def sinusoidal(self, t):
        height = (self.height_limit[1] - self.height_limit[0]) / 2 * np.sin(2*np.pi*t/self.T) + (self.height_limit[1] + self.height_limit[0]) / 2
        self.calc_theta_from_height(height)
        return self.theta1, self.theta1 + self.theta2

    def get_bezier_points(self, num_points=100):
        points = []
        for i in range(num_points+1):
            scale = i / num_points
            points.append(sum([self.c[i] * ((1-scale)**(self.order-i-1)) * (scale**i) for i in range(self.order)]))
        return points

    def linear_interpoolation(self, t, points):
        points.sort(key=lambda point: point[0])
        for i in range(len(points)-1):
            if points[i][0] <= t and t <= points[i+1][0]:
                return points[i][1] + (points[i+1][1] - points[i][1]) * (t - points[i][0]) / (points[i+1][0] - points[i][0])
        return points[-1][1]

    def bezier(self, t):
        period = t // self.T
        t = t % self.T
        if period % 2 == 0:
            scale = t / self.T
            points = self.get_bezier_points()
            height = self.linear_interpoolation(scale, points)*(self.height_limit[1] - self.height_limit[0]) + self.height_limit[0]
        else :
            height = self.height_limit[1] - (self.height_limit[1] - self.height_limit[0]) * t / self.T
        self.calc_theta_from_height(height)
        return self.theta1, self.theta1 + self.theta2
    
    
    def plot(self, func, title):
        t = np.linspace(0, self.T*6, 1000)
        theta1, theta2 = [], []
        for i in t:
            theta1_, theta2_ = func(i)
            theta1.append(theta1_)
            theta2.append(theta2_)
        plt.plot(t, theta1, label=f'{title} theta 1')
        plt.plot(t, theta2, label=f'{title} theta 2')
        plt.xlabel('time [s]')
        plt.ylabel('angle [rad]')
        plt.title(title)
        plt.legend()
        plt.show()

if __name__ == '__main__':
    controller = CuryController()
    controller.plot(controller.linear, 'Linear')
    controller.plot(controller.sinusoidal, 'Sinusoidal')
    controller.plot(controller.bezier, 'Bezier')