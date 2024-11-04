import numpy as np
import math


# unit is different with bezier_2d, in bezier_3d the units are meter and second

def bernstein_poly(i, n, t):
    return math.comb(n, i) * ((1 - t) ** (n - i)) * (t ** i)

def bezier_position_3d(t, control_points):
    n = len(control_points) - 1
    position = np.zeros(3)  # Initialize the position vector for 3D

    for i in range(n + 1):
        # Bernstein basis function
        basis = bernstein_poly(i, n, t)

        # Update the position vector
        position += basis * np.array(control_points[i])

    return position

def control_points_parametric_3d(Tswing, xLength, yLength, n, Hswing, init_point):
    Vx_desired = xLength / Tswing
    Vy_desired = yLength / Tswing

    control_points = [
        (-xLength / 2, -yLength / 2, 0),  # c0
        (-xLength / 2 - Vx_desired / (n + 1) * Tswing, -yLength / 2 - Vy_desired / (n + 1) * Tswing, 0),  # c1
        (-xLength / 2 - Vx_desired / (n + 1) * 2 * Tswing, -yLength / 2 - Vy_desired / (n + 1) * 2 * Tswing, Hswing),  # c2
        (-xLength / 2 - Vx_desired / (n + 1) * 2 * Tswing, -yLength / 2 - Vy_desired / (n + 1) * 2 * Tswing, Hswing),  # c3
        (-xLength / 2 - Vx_desired / (n + 1) * 2 * Tswing, -yLength / 2 - Vy_desired / (n + 1) * 2 * Tswing, Hswing),  # c4
        (0, 0, Hswing),  # c5
        (0, 0, Hswing),  # c6
        (0, 0, Hswing * 1.2),  # c7
        (xLength / 2 + 2 * Vx_desired * Tswing / (n + 1), yLength / 2 + 2 * Vy_desired * Tswing / (n + 1), Hswing * 1.2),  # c8
        (xLength / 2 + 2 * Vx_desired * Tswing / (n + 1), yLength / 2 + 2 * Vy_desired * Tswing / (n + 1), Hswing * 1.2),  # c9
        (xLength / 2 + Vx_desired * Tswing / (n + 1), yLength / 2 + Vy_desired * Tswing / (n + 1), 0),  # c10
        (xLength / 2, yLength / 2, 0)  # c11
    ]
    
    control_points = [
        (
            (x + xLength / 2)  + init_point[0],
            (y + yLength / 2)  + init_point[1],
            z  + init_point[2]
        )
        for x, y, z in control_points
    ]

    return control_points


class Trajectory_Generator:
    def __init__(self, T_swing, xLength, yLength, Height, init_point):      
        self.T_swing = T_swing
        self.xLength = xLength
        self.yLength = yLength
        self.Height = Height
        self.control_points = []
        self.init_point = init_point
        
        self.previous_time = 0
        self.previous_x = self.init_point[0]
        self.previous_y = self.init_point[1]
        self.previous_z = self.init_point[2]
        
        self.start_time = 0

    def get_control_points(self,swing_start_time):
        self.control_points = control_points_parametric_3d(
            self.T_swing,
            self.xLength,
            self.yLength,
            11,
            self.Height,
            self.init_point
        )
        self.start_time = swing_start_time     
        return self.control_points

    def get_Bezier_curve(self,swing_current_time): #t must go from 0 to 1   
        current_time = swing_current_time
        t = current_time - self.start_time
        normalized_time = t / self.T_swing
        
        if normalized_time > 1:
            normalized_time = 1  # if not, the trajectory will explode
            
        dt = normalized_time - self.previous_time
        
        position = bezier_position_3d(normalized_time, self.control_points)
        x, y, z = position
        
        if dt != 0:
            vx = (x - self.previous_x) / dt
            vy = (y - self.previous_y) / dt
            vz = (z - self.previous_z) / dt
        else:
            vx = vy = vz = 0
        
        self.previous_x = x
        self.previous_y = y
        self.previous_z = z
        self.previous_time = normalized_time
        
        return [x, y, z], [vx, vy, vz]