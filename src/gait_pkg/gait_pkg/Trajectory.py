import numpy as np
import math
import time


# Function to compute Bernstein basis polynomial
def bernstein_poly(i, n, t):
    return math.comb(n, i) * ((1 - t) ** (n - i)) * (t ** i)

def bezier_position(t, control_points):
    n = len(control_points) - 1
    position = [0, 0]  # Initialize the position vector

    for i in range(n + 1):
        # Bernstein basis function
        basis = bernstein_poly(i, n, t)

        # Update the position vector
        position[0] += basis * control_points[i][0]
        position[1] += basis * control_points[i][1] 

    
    return position

def control_points_parametric(Tswing, Length, n, Hswing, init_point):
    Vx_desired = Length/Tswing
    Tstance = Tswing    
    
    #
    control_points = [
        (-Length/2, 0),  # c0
        (-Length/2 - Vx_desired/(n + 1) * Tswing, 0),  # c1
        (-Length/2  - Vx_desired/(n + 1) * 2*Tswing, 	Hswing),  # c2
        (-Length/2  - Vx_desired/(n + 1) * 2*Tswing, 	Hswing),  # c3
        (-Length/2  - Vx_desired/(n + 1) * 2*Tswing, 	Hswing),  # c4
        (0, 	Hswing),  # c5
        (0, 	Hswing),  # c6
        (0, 	Hswing*1.2),  # c7
        (Length/2+2*Vx_desired*Tswing/(n+1), Hswing*1.2),  # c8
        (Length/2+2*Vx_desired*Tswing/(n+1), Hswing*1.2),  # c9
        (Length/2+Vx_desired*Tswing/(n+1), 0),  # c10
        (Length/2, 0)  # c11
    ]
    control_points = [((x + Length/2)/1000+ init_point[0], (y/1000+init_point[1])) for x, y in control_points]

    return control_points

class Trajectory_Generator:
    def __init__(self, T_swing,Length, Height, Tilt):      
         
        self.previous_time = 0
        self.previous_x = 0
        self.previous_y = 0  
        self.previous_z = 0        
        self.Height = Height
        self.Length = Length
        self.step = Length/3000
        self.T_swing = T_swing
        self.control_points = []
        self.start = []
        self.stand_end_point = []
        self.y_tilt = Tilt
        
        self.start_time = 0
        
        
    def get_control_points(self, start_point):
        self.control_points = control_points_parametric(self.T_swing,self.Length, 11, self.Height, (start_point[0], start_point[2])) 
        self.previous_time = time.time()
        self.start_time = self.previous_time
        self.previous_x = start_point[0]
        self.previous_y = start_point[1]
        self.previous_z = start_point[2]
        self.start = start_point
        
        self.stand_end_point = [start_point[0],start_point[1], start_point[2]]
        
        #self.tilt_end_point = [self.x_tilt + start_point[0],start_point[1], start_point[2]]
        return self.control_points
    def get_Bezier_curve(self): #t must go from 0 to 1   
        
       
        current_time = time.time()
        
        t = current_time - self.start_time
        
        normalized_time = t/self.T_swing
        
        if normalized_time>1:
            normalized_time = 1 #if not, the trajectory will explode
            
        dt = normalized_time - self.previous_time
        
        x, z = bezier_position(normalized_time, self.control_points)
        #y = self.start[1]
        
          
        y = self.start[1]
        if dt != 0:
            vx = (x - self.previous_x)/dt
            vz = (z - self.previous_z)/dt
            vy = 0
        else: 
            vx = 0
            vz = 0
            vy = 0
        
        
        self.previous_x = x
        self.previous_z = z
        self.previous_y = y
        self.previous_time = normalized_time
        
        return [x, y, z], [vx, vy, vz]

    def get_Stand_curve(self):
        current_time = time.time()
        
        t = current_time - self.start_time
        
        normalized_time = t/self.T_swing
        
        if normalized_time>1:
            normalized_time = 1 #if not, the trajectory will explode
            
        dt = normalized_time + self.previous_time  
        x = self.start[0] + normalized_time*-self.Length/3000  #For Crawl Gait
        y = self.stand_end_point[1]
        z = self.stand_end_point[2]
        
        if dt != 0:
            vx = (x - self.previous_x)/dt
        else:
            vx = 0
        vz = 0
        vy = 0
        self.previous_x = x
        self.previous_z = z
        self.previous_time = normalized_time
        
        return (x,y, z), (vx, vy, vz)
    
    
    def get_Stand_Curve_Sine(self, amplitude):
        current_time = time.time()
        
        t = current_time - self.start_time
        normalized_time = t/self.T_swing
        
        if normalized_time>1:
            normalized_time = 1 #if not, the trajectory will explode
        dt = normalized_time + self.previous_time  
        
        
        period = 2*self.Length/3000
        x_wave = normalized_time*self.Length/3000
        wave = -amplitude * np.sin(2 * np.pi * x_wave / period)
        
        x = self.start[0] + normalized_time*-self.Length/3000  #For Crawl Gait
        y = self.stand_end_point[1]
        z = self.stand_end_point[2] + wave
        
        if dt != 0:
            vx = (x - self.previous_x)/dt
        else:
            vx = 0
        vz = 0
        vy = 0
        self.previous_x = x
        self.previous_z = z
        self.previous_time = normalized_time
        
        return (x,y, z), (vx, vy, vz)
    def get_Linear_curve(self, direction,i, Tswing):
        
        current_time = time.time()
        
        t = current_time - self.start_time
        normalized_time = t/Tswing
        
        if normalized_time>1:
            normalized_time = 1 #if not, the trajectory will explode
        dt = normalized_time - self.previous_time  
        
        Lambda = [1,-1,1,-1]
        
        end = Lambda[i]*0.078+ direction*self.y_tilt
       
        y = self.start[1] + normalized_time*(end- self.start[1])

        x = self.start[0] 
        z = self.start[2]
        
        #print(self.start[1])
        vx = 0
        vz = 0
        if dt != 0:
            vy = (y - self.previous_y)/dt
        else:
            vy = 0
        
        self.previous_x = x
        self.previous_y = y
        self.previous_z = z
        self.previous_time = normalized_time
        
        return (x,y,z), (vx,vy,vz)
    
    def CPG_y (self, t, quadrant, i): #t must go from 0 to 1       
        
        factor = 2
        normalized_time = t/self.T_swing*factor        
        dt = normalized_time - self.previous_time         
        
        #y = self.start[1]
        
        period = 4*self.step
                
        
        
        if normalized_time <= 1:
            x_wave = normalized_time*self.step + quadrant*self.step
        else:
            x_wave = self.step + quadrant*self.step
        
        wave = self.y_tilt * np.sin(2 * np.pi * x_wave / period)
        #print(wave)
        Lambda = [1,-1,1,-1]       
        
        y = Lambda[i]*0.078 + wave
        
        
        x = self.start[0] 
        z = self.start[2]
      
        if dt != 0:            
            vy = (y - self.previous_y)/dt
        else: 
            vy = 0
            
        vx = 0        
        vz = 0
            
        
        
        self.previous_x = x
        self.previous_z = z
        self.previous_y = y
        self.previous_time = normalized_time
        
        return y, vz
        

        
        

