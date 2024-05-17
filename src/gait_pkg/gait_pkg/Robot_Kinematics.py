import numpy as np
from math import sqrt, atan2, acos, sin, cos, pi

class RobotLeg:
    def __init__(self, leg):        
        """
        Initialize a RobotLeg object.

        Parameters:
        - leg (str): The identifier of the leg. Can be "FL", "FR", "BL", or "BR".
        """
                
        # Initialize a robot leg with given leg identifier
        self.leg = leg
        # Determine the index of the leg based on the identifier
        self.index = {"FL": 0, "FR": 1, "BL": 2, "BR": 3}[leg]
        
        # Define lengths of arm segments and orientation
        self.a0 = 0.078 # Shoulder-Hip Link [m]
        self.a1 = 0.307 # Hip-Knee Link [m]
        self.a2 = 0.307 # Knee-Toe Link [m]
        
        #Local Leg Coordinates 
        self.body_width = 0 # 0.088m 
        self.body_length = 0 # 0.3227m
        
        self.Lambda = [1, -1, 1, -1][self.index] # Body_length sign
        self.delta = [1, 1, -1, -1][self.index] # Shoulder-Hip Link and body_width sign  
        
        
        self.security_position = [0.0, 60.8, -121.5] #

    def T_Jacobian(self, theta1_deg, theta2_deg, theta3_deg):
        """
        Calculate the Jacobian matrix for the leg based on given joint angles
        
        Parameters: 
        - Shoulder Angle [deg]
        - Hip Angle [deg]
        - Knee Angle [deg]
        
        Return: 
        - 3x3 Transposed Jacobian Matrix
        """
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)
        theta3 = np.radians(theta3_deg)
        Fc0 = cos(theta1)
        Fs0 = sin(theta1)
        Fc1 = cos(theta2)
        Fs1 = sin(theta2)
        Fc12 = cos(theta2 + theta3)
        Fs12 = sin(theta2 + theta3)

        T_Jacobian = np.zeros((3, 3))
        T_Jacobian[0][1] = -self.Lambda * self.a0 * Fs0 + self.a2 * Fc0 * Fc12 + self.a1 * Fc1 * Fc0
        T_Jacobian[0][2] = self.Lambda * self.a0 * Fc0 + self.a2 * Fs0 * Fc12 + self.a1 * Fs0 * Fc1

        T_Jacobian[1][0] = -self.a1 * Fc1 - self.a2 * Fc12
        T_Jacobian[1][1] = -self.a2 * Fs0 * Fs12 - self.a1 * Fs1 * Fs0
        T_Jacobian[1][2] = self.a2 * Fc0 * Fs12 + self.a1 * Fc0 * Fs1

        T_Jacobian[2][0] = -self.a2 * Fc12
        T_Jacobian[2][1] = -self.a2 * Fs0 * Fs12
        T_Jacobian[2][2] = self.a2 * Fc0 * Fs12

        return T_Jacobian

    def Forward_Kinematics_Local(self, theta1_deg, theta2_deg, theta3_deg):
        """
        Calculate the forward kinematics for the leg based on given joint angles
        
        Parameters: 
        - Shoulder Angle [deg]
        - Hip Angle [deg]
        - Knee Angle [deg]
        
        Return: 
        - [X, Y, Z] coordinates in m
        """
        theta1 = np.radians(theta1_deg)
        theta2 = np.radians(theta2_deg)
        theta3 = np.radians(theta3_deg)
        s0 = sin(theta1)
        s1 = sin(theta2)
        c0 = cos(theta1)
        c1 = cos(theta2)
        s12 = sin(theta2 + theta3)
        c12 = cos(theta2 + theta3)
        x = -self.a1 * s1 - self.a2 * s12 + self.delta * self.body_length              
        y = self.Lambda*(self.a0 * c0 + self.body_width) + self.a2 * s0 * c12 + self.a1 * c1 * s0
        z = self.Lambda * self.a0 * s0 - self.a2 * c0 * c12 - self.a1 * c0 * c1
        
        
        return [x, y, z]

    def Inverse_Kinematics_Local(self, positions):
        """
        Calculate the inverse kinematics for the leg based on given end effector positions
        
        Parameters: 
        - [X, Y, Z] cartesian position based on the leg base coordinates [m]
        
        Return: 
        - [Theta1, Theta2, Theta3] [m]
        """
        
        
        #This is an Adaptation from Soren and Simon Coordinates
        z, x, y = positions
        identifier = [1,0,3,2]
        i = identifier[self.index]   
        
        try:
            AG = sqrt(x ** 2 + y ** 2 - self.a0 ** 2)
            AC = sqrt(AG ** 2 + z ** 2)
            
            theta1 = (-atan2(y, x) - atan2(AG, -self.a0 * (-1) ** i)) * 180 / pi
            theta3 = 180 + (acos(-(AC ** 2 - self.a1 ** 2 - self.a2 ** 2) / (2 * self.a1 * self.a2)) - pi) * 180 / pi
            theta2 = (atan2(z, AG)) * 180 / pi - acos(AC ** 2 / (2 * self.a1 * AC)) * 180 / pi
                
            #Transform angles
                        
            theta1 = -theta1
            theta2 = -theta2
            theta3 = - (180 - theta3)
            
            if theta1 >= 180:
                theta1 = -(360-theta1)
            if theta2 >= 180:
                theta2 = -(360-theta2)
            if theta3 >= 180:
                theta3 = -(360-theta3)
        except:
            
            raise ValueError("No solution found for inverse kinematics. The position may be unreachable.")
        
        return [theta1, theta2, theta3]
    
    

if __name__ == "__main__":
    leg = RobotLeg("FR")
    print(leg.Inverse_Kinematics_Local([0, 0.078, -0.3]))
    print(leg.Forward_Kinematics_Local(0, 45, -120))
    
    print(leg.Inverse_Kinematics(1,[0, 0.078, -0.3]))
    print(leg.Forward_Kinematics(1,[0, 45, -120]))