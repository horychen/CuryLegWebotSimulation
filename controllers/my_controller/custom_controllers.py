import numpy as np
from matplotlib import pyplot as plt

class CuryController:
    def __init__(self, **kwargs):
        """Parameters:
        L1_length: the length of the L1
        L2: the length of the L2
        four_bar_linkage: the length of the four bar linkage. [a, b, c, d, x, y]
        simulation_offset: the offset of the simulation. [hip angle offset, shank angle offset, hip motor position offset, shank motor position offset]. unit: [rad, rad, m, m]
        real_offset: the offset of the real robot. [hip angle offset, shank angle offset, hip motor position offset, shank motor position offset]. unit: [rad, rad, m, m]
        simulation_direction: the direction of the simulation. [hip angle direction, shank angle direction, hip motor position direction, shank motor position direction]
        """
        self.L1 = kwargs.get('L1_length', 0.4)
        self.L2 = kwargs.get('L2_length', 0.39495)
        # self.simulation_offset = kwargs.get('simulation_offset', [-0.5564, 0.4937, 0.0595, -0.046])
        # self.theory_offset = kwargs.get('real_offset', [2.2440, 3.0384, 0.0731, 0.1503])
        self.four_bar_linkage = kwargs.get('four_bar_linkage', [0.08, 0.13398, 0.15307, 0.05166, 0.11076, 0.19203])
        self.four_bar_gravity = kwargs.get('four_bar_gravity', [0.1743, 0.2157, 0.0878, 0.0])
        self.simulation_offset = kwargs.get('simulation_offset', [-0.3854, 0.4937, 0.0430, -0.046])
        self.theory_offset = kwargs.get('real_offset', [2.3935, 3.0384, 0.0873, 0.1503])
        self.simulation_direction = kwargs.get('simulation_direction', [1, -1, -1, 1])
        self.theta_to_angle_offest = kwargs.get('theta_to_angle_offest', [1.7279, 2.5251])
        self.theta1, self.theta2 = 0.0, 0.0
        self.dtheta1, self.dtheta2 = 0.0, 0.0
        self.ddtheta1, self.ddtheta2 = 0.0, 0.0

    def theorydistance_to_simulationdistance(self, theory_distance):
        simulation_distance = [(theory_distance[0] - self.theory_offset[2])*self.simulation_direction[2] + self.simulation_offset[2],
                                (theory_distance[1] - self.theory_offset[3])*self.simulation_direction[3] + self.simulation_offset[3]]
        return simulation_distance
    
    def simulationjointAngle_2_theoryjointAngle(self, simulation_joint_angle):
        theory_joint_angle = [(simulation_joint_angle[0] - self.simulation_offset[0])*self.simulation_direction[0] + self.theory_offset[0],
                            (simulation_joint_angle[1] - self.simulation_offset[1])*self.simulation_direction[1] + self.theory_offset[1]]
        return theory_joint_angle
    
    def theoryjointAngle_to_simulationjointAngle(self, theory_joint_angle):
        simulation_joint_angle = [(theory_joint_angle[0] - self.theory_offset[0])*self.simulation_direction[0] + self.simulation_offset[0],
                                (theory_joint_angle[1] - self.theory_offset[1])*self.simulation_direction[1] + self.simulation_offset[1]]
        return simulation_joint_angle
    
    def motorPosition_to_jointAngle(self, distance):
        jointAngle = []
        for distance_motor in distance:
            gama = np.arccos((distance_motor**2 + (self.four_bar_linkage[4] - self.four_bar_linkage[3])**2 + self.four_bar_linkage[5]**2 - self.four_bar_linkage[2]**2)
                                / (2*distance_motor*np.sqrt((self.four_bar_linkage[4] - self.four_bar_linkage[3])**2 + self.four_bar_linkage[5]**2)))
            beta = np.arctan((self.four_bar_linkage[4] - self.four_bar_linkage[3])/self.four_bar_linkage[5])
            
            cx = self.four_bar_linkage[4] - distance_motor*np.sin(gama+beta)
            cy = self.four_bar_linkage[5] - distance_motor*np.cos(gama+beta)
            
            distance_ac = np.sqrt(cx**2 + cy**2)
            
            angle_bac = np.arccos((distance_ac**2 + self.four_bar_linkage[0]**2 - self.four_bar_linkage[1]**2)/(2*distance_ac*self.four_bar_linkage[0]))
            if cx > 0: angle_dac = np.arctan(cy/cx)
            elif cx == 0: angle_dac = np.pi/2
            else: angle_dac = np.arctan(cy/cx) + np.pi
            
            jointAngle.append(angle_bac + angle_dac)
        if len(jointAngle) == 1: return jointAngle[0]
        return jointAngle

    def jointAngle_to_motorPosition(self, jointAngle):
        distance = []
        for angle in jointAngle:
            bx = self.four_bar_linkage[0]*np.cos(angle)
            by = self.four_bar_linkage[0]*np.sin(angle)
            
            distance_bd = np.sqrt((self.four_bar_linkage[3] - bx)**2 + by**2)
            if bx < self.four_bar_linkage[3]: angle_bda = np.arctan(by/(self.four_bar_linkage[3] - bx))
            elif bx == self.four_bar_linkage[3]: angle_bda = np.pi/2
            else: angle_bda = np.arctan(by/(self.four_bar_linkage[3] - bx)) + np.pi
            
            angle_bdc = np.arccos((distance_bd**2 + self.four_bar_linkage[2]**2 - self.four_bar_linkage[1]**2)/(2*distance_bd*self.four_bar_linkage[2]))
            angle_adc = angle_bda + angle_bdc
            
            cx = self.four_bar_linkage[3] - self.four_bar_linkage[2]*np.cos(angle_adc)
            cy = self.four_bar_linkage[2]*np.sin(angle_adc)
            
            distance.append(np.sqrt((cx - self.four_bar_linkage[4])**2 + (cy - self.four_bar_linkage[5])**2))
        if len(distance) == 1: return distance[0]
        return distance

    def calc_theta_from_height(self, height, dheight, ddheight):
        self.theta1 = np.arccos((self.L1**2 + height**2 - self.L2**2) / (2*self.L1*height))
        self.theta2 = np.arccos((self.L2**2 + height**2 - self.L1**2) / (2*self.L2*height))
        self.dtheta1 = - dheight / (self.L1 * np.sin(self.theta1) + self.L2 * np.sin(self.theta2) * self.L2 * np.cos(self.theta1) / (self.L1 * np.cos(self.theta2)))
        self.dtheta2 = self.dtheta1 * self.L2 * np.cos(self.theta1) / (self.L1 * np.cos(self.theta2))
        self.ddtheta2 = (-self.L1 * self.theta1**2 + 
                         (self.L1**2 * np.sin(self.theta1)*np.sin(self.theta2)) / (self.L2 * np.cos(self.theta1)) * self.dtheta2**2 - 
                         (self.L1*np.sin(self.theta1)**2 * self.dtheta1**2) / np.cos(self.theta1) -
                         self.L2 * np.cos(self.theta2) * self.dtheta2**2 - ddheight
                        ) / ((self.L1**2 * np.cos(self.theta1) * np.sin(self.theta1) /  (self.L2 * np.cos(self.theta1))) + self.L2*np.sin(self.theta2))
        self.ddtheta1 = (- self.L1 * np.sin(self.theta2) * self.dtheta2**2 + 
                         self.L1*np.cos(self.theta2)*self.ddtheta2 + 
                         self.L2*np.sin(self.theta1)*self.dtheta1**2) / (self.L2*np.cos(self.theta1))
    
    def get_inertia(self) -> list:
        """From theta1 and theta2 to calc inertia
        """
        pass

    def calc_force_from_theta(self, ):
        inertia = self.get_inertia()
        torques = [inertia*self.ddtheta1, inertia*self.ddtheta2]
        alphas = [self.theta1+self.theta_to_angle_offest[0], self.theta2+self.theta_to_angle_offest[2]-self.theta1]
        a,b,c,d,x0,y0 = self.four_bar_linkage
        betas = [0.0, 0.0]              # TODO:
        zax, zay = 0.0, 0.0             # TODO:
        
        def alpha_to_theta(alpha):
            bx = a * np.cos(alpha)
            by = a * np.sin(alpha)
            
            bc = np.sqrt((bx-d)**2 + by**2)
            
            if bx < d: angle_adb = np.arctan((d-bx)/by)
            elif bx == d: angle_adb = np.pi/2
            else: angle_adb = np.pi/2 + np.arctan((bx-d)/by)
            
            angle_bdc = np.arccos((bc**2 + c**2 -b**2)/(2*bc*c))
            
            return angle_adb + angle_bdc
        
        def get_dtheta_dalpha(theta, alpha):
            return (2 * a * c * np.sin(alpha + theta) - 2 * a * d * np.sin(theta)) / (
                    2 * c * d * np.sin(theta) - 2 * a * c * np.sin(alpha + theta))
        
        def normOC(theta):
            return np.sqrt((d - c * np.cos(theta) - x0) ** 2 + (c * np.cos(theta) - y0) ** 2)
        
        F_result = []
        # TODO: wrong answer
        for index in range(2):
            alpha = alphas[index]
            torque = torques[index]
            beta = betas[index]
            theta = alpha_to_theta(alphas[index])
            dtheta_dalpha = get_dtheta_dalpha(theta, alphas[index])
            
            FC_section = (c * np.sin(theta) * dtheta_dalpha * (d - c * np.cos(theta) - x0)
                        + c * np.cos(theta) * dtheta_dalpha * (c * np.sin(theta) - y0)) / normOC(theta)

            Ga_section = self.four_bar_gravity[0] * (np.cos(beta) * (-zax * np.sin(alpha) - zay * np.cos(alpha)) +
                            np.sin(beta) * (zax * np.cos(alpha) - zay * np.sin(alpha)))

            Gb_section = self.four_bar_gravity[1] * (np.cos(beta) * (- a * np.sin(alpha) + c * np.sin(theta)) * dtheta_dalpha / 2 +
                            np.sin(beta) * (a * np.cos(alpha) + c * np.cos(theta)) * dtheta_dalpha / 2)

            Gc_section = self.four_bar_gravity[2] * (np.cos(beta) * (-c * np.sin(theta) / 2 * dtheta_dalpha) +
                            np.sin(beta) * (c * np.cos(theta) / 2 * dtheta_dalpha))

            Gd_section = 0.0
            F = (torque - Ga_section - Gb_section - Gc_section - Gd_section) / FC_section
            F_result.append(F)
        
        return F_result


    def LinearController(self):
        pass

    def SinController(self):
        pass

    def BezierController(self):
        pass


class controllerTest(CuryController):
    def __init__(self):
        super().__init__()

    def motorPosition_to_jointAngleAngleTest(self):
        def test(simulation_distance):
            print(f"simulation_distance: {simulation_distance}", end="\t")
            theory_distance = self.simlutiondistance_to_theorydistance(simulation_distance)
            theory_joint_angle = self.motorPosition_to_jointAngle(theory_distance)
            simulation_joint_angle = self.theoryjointAngle_to_simulationjointAngle(theory_joint_angle)
            print(f"Simulation joint angle: {simulation_joint_angle}", end="\n")
            return simulation_joint_angle
        test([0.0590, -0.046])
        test([-0.053, 0.080])
        test([0.06774208142350613, -0.0005323936956123775])
        test([0.06774208142350616, -0.0005323936956124625])
        pass
    
    def jointAngle_to_motorPositionTest(self):
        def test(jointAngle):
            print(f"jointAngle: {jointAngle}", end="\t")
            theory_joint_angle = self.simulationjointAngle_2_theoryjointAngle(jointAngle)
            theory_distance = self.jointAngle_to_motorPosition(theory_joint_angle)
            simulation_distance = self.theorydistance_to_simulationdistance(theory_distance)
            print(f"simulation distance: {simulation_distance}", end="\n")
            return simulation_distance
        test([-0.5548745824469792, 0.4943889283876448])
        test([0.6262192355214469, -1.1134681676207046])
        test([0.06355590695776303, -0.9264912223065996])
        test([-0.6576364648991879, -0.020999170627387542])
        pass
    
    def plot_compare_distance(self, data):
        data_size = len(data['hip_joint_angle'])
        
        hip_joint_angle = data['hip_joint_angle']
        shank_joint_angle = data['shank_joint_angle']
        hip_motor_position = data['screw_shank_joint_sensor']
        shank_motor_position = data['screw_hip_joint_sensor']
        
        calc_distance = []
        
        for index in range(data_size):
            theory_angle = self.simulationjointAngle_2_theoryjointAngle([hip_joint_angle[index], shank_joint_angle[index]])
            theory_distance = self.jointAngle_to_motorPosition(theory_angle)
            simulation_distance = self.theorydistance_to_simulationdistance(theory_distance)
            calc_distance.append(simulation_distance)
        
        plt.figure()
        plt.plot(hip_motor_position, label='hip motor position', markersize=0.5, linewidth=1)
        plt.plot([distance[0] for distance in calc_distance], '-o', label='calc hip motor position', markersize=0.5, linewidth=0.5)
        plt.plot(shank_motor_position, label='shank motor position', markersize=0.5, linewidth=1)
        plt.plot([distance[1] for distance in calc_distance], '-o', label='calc shank motor position', markersize=0.5, linewidth=0.5)
        plt.ylabel('motor to bar node [m]')
        plt.title('Compare simulation motor position to theory motor position')
        plt.legend()
        plt.grid()
        
        
    def plot_compare_angle(self, data):
        data_size = len(data['hip_joint_angle'])
        
        hip_joint_angle = data['hip_joint_angle']
        shank_joint_angle = data['shank_joint_angle']
        hip_motor_position = data['screw_shank_joint_sensor']
        shank_motor_position = data['screw_hip_joint_sensor']
        
        print(hip_joint_angle[0], shank_joint_angle[0], hip_motor_position[0], shank_motor_position[0])
        
        calc_angle = []
        
        for index in range(data_size):
            theory_distance = self.simlutiondistance_to_theorydistance([hip_motor_position[index], shank_motor_position[index]])
            theory_joint_angle = self.motorPosition_to_jointAngle(theory_distance)
            simulation_joint_angle = self.theoryjointAngle_to_simulationjointAngle(theory_joint_angle)
            calc_angle.append(simulation_joint_angle)
        
        plt.figure()
        plt.plot(hip_joint_angle, label='hip joint angle', markersize=0.5, linewidth=1)
        plt.plot([angle[0] for angle in calc_angle], '-o', label='calc hip joint angle', markersize=0.5, linewidth=0.5)
        plt.plot(shank_joint_angle, label='shank joint angle', markersize=0.5, linewidth=1)
        plt.plot([angle[1] for angle in calc_angle], '-o', label='calc shank joint angle', markersize=0.5, linewidth=0.5)
        plt.ylabel('joint angle [rad]')
        plt.title('Compare simulation joint angle to theory joint angle')
        plt.legend()
        plt.grid()


if __name__ == '__main__':
    tester = controllerTest()
    # tester.motorPosition_to_jointAngleAngleTest()
    # tester.jointAngle_to_motorPositionTest()