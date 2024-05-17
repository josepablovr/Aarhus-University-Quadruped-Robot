from . Robot_Kinematics import RobotLeg
from . Trajectory import Trajectory_Generator
import time
from enum import Enum
import numpy as np

class Robot_Movement:    

    def __init__(self):
       
        self.desired_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.desired_velocities = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.stand_height = 0
        self.swing_step = 0.15
        self.swing_time = 3 
        self.swing_height = 0.085
        self.tilt_time = 3 # 1.5
        self.phase_time_swing = 2 + self.swing_time  #1.5
        self.phase_time_tilt = 2 + self.tilt_time #2
        self.tilt_distance = 0.06
        
        self.max_speed = 200
        self.restart_speed = 20
        self.speed = self.max_speed
        self.body_height = -0.4
        self.legs = [RobotLeg("FL"), RobotLeg("FR"), RobotLeg("BL"), RobotLeg("BR")]
        self.Trajectories = [Trajectory_Generator(self.swing_time, self.swing_step*1000, self.swing_height*1000, self.tilt_distance) for _ in range(4)]
        self.states = [0, 0, 0, 0]
        self.elapsed_time = 0
        self.dt = 0
        self.Lambda = [1,-1,1,-1]
        self.previous_time = 0
        self.phase_time_start = 10
        self. z_offset = -self.swing_step/2 -0.05
        self.simulation = False
        self.movement_started = False
        
        
    
    def start(self):        
        
              
        if self.movement_started == False:            
            dt = 0.01
            self.speed = self.restart_speed
            self.states = [0, 0, 0, 0]
            self.movement_started = True
           
            
        else:
            dt = time.time() - self.previous_time
        
        self.previous_time = time.time()
        if self.simulation == False:
            self.elapsed_time += dt
        
        #print(self.elapsed_time)
        
        

        #desired_positions = [(z_offsets[0], x+x_tilt, y1), (z_offsets[1], -x+x_tilt, y1), (z_offsets[2], x+x_tilt, y1), (z_offsets[3], -x+x_tilt, y1)]
        for i in range(4):
            z_offsets = [1/3*self.swing_step+self.z_offset, self.swing_step+self.z_offset, 0+self.z_offset, 2/3*self.swing_step+self.z_offset]            
            self.desired_positions[i] = [z_offsets[i], self.Lambda[i]*0.078, self.body_height]
            
        
        
        if self.elapsed_time >= self.phase_time_start:               
            print("Finished Start")
            self.elapsed_time = 0
            self.movement_started = False
            self.speed = self.max_speed
            return 1
        
        
        
    def Tilt(self, side):
        '''
        side: right=1 or left=-1
        '''   
       
        if self.movement_started == False: 
            self.movement_started = True 
            dt = 0.01
            self.max_speed = 100
            self.states = [0,0,0,0]
            for i in range(4):
                x, y, z = self.desired_positions[i]        
                   
                self.Trajectories[i].get_control_points((x, y, z))
            
            
           
        else:
           
            dt = time.time() - self.previous_time
        
            self.previous_time = time.time()
            if self.simulation == False:
                self.elapsed_time += dt
            
          
                
            
            for i in range(4): 
                positions, velocities = self.Trajectories[i].get_Linear_curve(side,i, self.tilt_time)
                
                self.desired_positions[i] = positions         
                self.desired_velocities[i] = velocities
            
            if self.elapsed_time >= self.phase_time_tilt:               
                print("Finished Tilting")
                self.elapsed_time = 0
                self.movement_started = False
                return 1
        return 0
                
        
    def Swing_Leg(self, leg):
        '''
        side: right=1 or left=-1
        '''   
        index = {"FL": 0, "FR": 1, "BL": 2, "BR": 3}[leg]
            
        if self.movement_started == False:   
                     
            dt = 0.01
            self.max_speed = 100
            self.states = [0,0,0,0]
            self.states[index] = 1
            self.movement_started = True
            for i in range(4):                
                x, y, z = self.desired_positions[i]                    
                self.Trajectories[i].get_control_points((x, y, z))
                
        else:
            dt = time.time() - self.previous_time
        
        self.previous_time = time.time()
        self.elapsed_time += dt
        
        for i in range(4): 
            if self.states[i] == 0:                        
                positions, velocities = self.Trajectories[i].get_Stand_Curve_Sine(self.stand_height)
            else: 
                positions, velocities = self.Trajectories[i].get_Bezier_curve()
            self.desired_positions[i] = positions         
            self.desired_velocities[i] = velocities
            
        
        
        if self.elapsed_time >= self.phase_time_swing:               
            print("Finished Swinging")
            self.elapsed_time = 0   
            self.movement_started = False
            return 1
        return 0
    
    def get_joint_position(self):
    
        joint_angles = []
        try:
            
            for e in range(4):               
                leg_angles = self.legs[e].Inverse_Kinematics_Local(self.desired_positions[e])
                joint_angles.append(leg_angles)
        except ValueError as e:
            self.speed = self.restart_speed
            # Handle the ValueError raised from the function
            print("An error occurred:", e)
            print("The robot position is set to the start point")
            joint_angles = []
            for e in range(4):
                leg_angles = self.legs[e].security_position
                joint_angles.append(leg_angles)
            
            
            
        return joint_angles
        
    def update(self):
        tic = time.time()
        self.dt = tic - self.elapsed_time

        if self.gait_state == self.CrawlStates.Start:
            self.set_start_state()

        # Add logic for other states here...

        self.elapsed_time += self.dt

    def set_start_state(self):
        # Logic for Start state
        pass

    # Define methods for other states...

if __name__ == "__main__":
    crawl_gait = Robot_Movement()
    
    #while crawl_gait.start() != 1:
        #time.sleep(0.1)
    
    while crawl_gait.Tilt(1) != 1:
        time.sleep(0.1)
        
    while crawl_gait.Swing_Leg("FL") != 1:
        time.sleep(0.1)
    
    
    print("End")