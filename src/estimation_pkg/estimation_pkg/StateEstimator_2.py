import numpy as np
from scipy.signal import butter, filtfilt, lfilter, lfilter_zi
from scipy.spatial.transform import Rotation as Rot
from . forward_kinematics import ForwardK1
from . forward_kinematics import CrossMatrix
from . forward_kinematics import R1
from tf_transformations import euler_from_quaternion, quaternion_from_euler
class StateEstimator:
    def __init__(self):
        pass
    
    def initialize(self, FL, FR, BL, BR, imu_data):
        
        FL_angles =  np.array(FL)
        FR_angles =  np.array(FR)
        BL_angles =  np.array(BL)
        BR_angles =  np.array(BR)
      
        
    
        imu_acc = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
        imu_ang_speed = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
          
        r_, p_, y_ = euler_from_quaternion(quaternions)
        self.acc_offset = 0
        R = R1(r_,p_,y_)
       
        a_imu = np.matrix([[imu_acc[0]], 
                        [imu_acc[1]],
                        [imu_acc[2]]])
        u_kf = R @ a_imu
        # g= 0 0 0 because g is already balestimated_contactsanced by other code
        
        self.ag = -u_kf
        #self.acc_offset = [-u_kf[0],-u_kf[1],-u_kf[2]]

        pss = np.vstack((FL_angles, FR_angles, BL_angles, BR_angles))
        #print(pss)
        self.lambda_ = [1, -1, 1, -1]
        self.delta = [1, 1, -1, -1]
        angle_current = np.zeros((3, 4))
        self.foot_position_B = np.zeros((3, 4))

        for i in range(4):
            t1 = np.radians(pss[i, 0])
            t2 = np.radians(pss[i, 1])
            t3 = np.radians(pss[i, 2])
            angle_current[:, i] = np.array([t1, t2, t3])
            x, y, z = ForwardK1(t1, t2, t3, self.lambda_[i], self.delta[i])
            self.foot_position_B[:, i] = np.array([x, y, z])

        # Xk
        self.position_last_B = np.matrix(self.foot_position_B)
        
        hight_initial = self.position_last_B[-1, :]
        average_hight_initial = - np.mean(hight_initial)

        Xk_32 = np.matrix ([[0, 0, average_hight_initial], [0, 0, 0]])
        Xk_63 = np.vstack((Xk_32, self.position_last_B.T))

        self.Xk = np.matrix(np.zeros((18, 1)))
        number = 0
        
        for i in range(6):
            for j in range(3):
                self.Xk[number] = Xk_63[i, j]
                number += 1

        # self.Pk
        self.Pk = np.matrix(np.eye((18)))

        # self.dt
        self.dt = 0.02

        # A_kf
        self.A_kf = np.matrix(np.eye(18))
        self.A_kf[0:3, 3:6] = self.dt * np.eye(3)

        # B_kf
        self.B_kf = np.matrix(np.zeros((18, 3)))
        self.B_kf[3:6, 0:3] = self.dt * np.eye(3)

        

      

        # PROCESS_NOISE
        self.PROCESS_NOISE = 0.01

        # B: body coord  O:world
        self.angle_current = np.matrix(np.zeros((3, 4)))
        self.foot_position_B = np.matrix(np.zeros((3, 4)))
        self.foot_position_O = np.matrix(np.zeros((3, 4)))
        self.vel_current_B = np.matrix(np.zeros((3, 4)))
        self.estimated_contacts = np.matrix(np.zeros((1,4)))

        # v_0  velocity initial
        self.v_0 = np.matrix([[0], [0], [0]])

        # forward kinematics parameters
        self.lambda_ = np.matrix([1, -1, 1, -1])
        self.delta = np.matrix([1, 1, -1, -1])
        
        # Create an 18x18 identity matrix
        self.Q0 = np.eye(18)

        # Replace specific blocks
        self.Q0[0:3, 0:3] = (self.dt / 20.0) * np.eye(3)
        self.Q0[3:6, 3:6] = (self.dt * 9.8 / 20.0) * np.eye(3)
        self.Q0[6:18, 6:18] = self.dt * np.eye(12)
        
        # Define the process noise values
        self.process_noise_pimu = 0.02  # Replace with the actual process noise for pimu
        self.process_noise_vimu = 0.02  # Replace with the actual process noise for vimu
        self.process_noise_pfoot = 1  # Replace with the actual process noise for pfoot
        
        
        # Define the sensor noise values
        self.sensor_noise_pimu_rel_foot = 0.001  # Replace with the actual sensor noise for pimu_rel_foot
        self.sensor_noise_vimu_rel_foot = 0.1  # Replace with the actual sensor noise for vimu_rel_foot
        self.sensor_noise_zfoot = 0.001  # Replace with the actual sensor noise for zfoot
        self.R0 = np.eye(28)
        
    def Run(self, FL, FR, BL, BR, leg_status, imu_data):
        
        
        FL_angles =  np.array(FL)
        FR_angles =  np.array(FR)
        BL_angles =  np.array(BL)
        BR_angles =  np.array(BR)
        
        imu_acc = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
        imu_ang_speed = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
          
        r_, p_, y_ = euler_from_quaternion(quaternions)
        
    
        imu_acc -= self.acc_offset
        
        #KALMAN FILTER LOOP
        

        a_imu = np.matrix([[imu_acc[0]], 
                        [imu_acc[1]],
                        [imu_acc[2]]])

        # joint angle

        pss = np.vstack((FL_angles, FR_angles, BL_angles, BR_angles))
        pss = np.asmatrix(pss)


        # O_R_B
        R_t = R1(r_,p_,y_)
        R = R_t

    
        for i in range(4):
            # Extract angles and convert to radians
            t1 = np.radians(pss[i, 0])
            t2 = np.radians(pss[i, 1])
            t3 = np.radians(pss[i, 2])

            # Compute angles and foot positions
            self.angle_current[:, i] = np.matrix([[t1], [t2], [t3]])
            x, y, z = ForwardK1(t1, t2, t3, self.lambda_[0, i], self.delta[0, i])


            self.foot_position_B[:, i] = np.matrix([[x], [y], [z]])
            
            # Rotate foot position to obtain foot position in O frame

            self.foot_position_O[:, i] = R @ np.matrix([[x], [y], [z]])
            
            # Compute current velocity in B frame
            self.vel_current_B[:, i] = (np.matrix([[x], [y], [z]]) - self.position_last_B[:, i]) / (self.dt)
            
            # Update last position
            self.position_last_B[:, i] = np.matrix([[x], [y], [z]])

        #print(self.vel_current_B)
        
        #  acc_body
        ab = a_imu

        #  omega_body
        wb = np.matrix([[imu_ang_speed[0]], [imu_ang_speed[1]], [imu_ang_speed[2]]])

        wb_cm = CrossMatrix(wb)

        u_kf = R @ ab + self.ag

        # Initialize Q_noise and R_noise matrices
        Q_noise = np.matrix(np.zeros((18, 18)))

        # version 2
        # most related to Q_noist 1e5 
        Q_noise[0:3, 0:3] =  self.PROCESS_NOISE * self.dt * np.matrix(np.eye(3)) / 20
        #Q_noise[2, 2] = 0.01 * self.dt / 20
        Q_noise[3:6, 3:6] =  10*9.8 * self.PROCESS_NOISE * self.dt * np.matrix(np.eye(3)) / 20
        
        
        # Create an 18x18 identity matrix
        Q_noise = np.eye(18)

        # Replace specific blocks of Q with scaled blocks from _Q0
        Q_noise[0:3, 0:3] = self.Q0[0:3, 0:3] * self.process_noise_pimu
        Q_noise[3:6, 3:6] = self.Q0[3:6, 3:6] * self.process_noise_vimu
        Q_noise[6:18, 6:18] = self.Q0[6:18, 6:18] * self.process_noise_pfoot


        R_noise =  self.PROCESS_NOISE * np.matrix(np.eye(28))
        
        # Create a 28x28 identity matrix
        R_noise = np.eye(28)

        # Replace specific blocks of R with scaled blocks from _R0
        R_noise[0:12, 0:12] = self.R0[0:12, 0:12] * self.sensor_noise_pimu_rel_foot
        R_noise[12:24, 12:24] = self.R0[12:24, 12:24] * self.sensor_noise_vimu_rel_foot
        R_noise[24:28, 24:28] = self.R0[24:28, 24:28] * self.sensor_noise_zfoot

        # Extract foot positions and compute their velocities in O frame
        P1_OV = R @ (wb_cm @ self.foot_position_B[:, 0] + self.vel_current_B[:, 0])
        P2_OV = R @ (wb_cm @ self.foot_position_B[:, 1] + self.vel_current_B[:, 1])
        P3_OV = R @ (wb_cm @ self.foot_position_B[:, 2] + self.vel_current_B[:, 2])
        P4_OV = R @ (wb_cm @ self.foot_position_B[:, 3] + self.vel_current_B[:, 3])

        foot_OV_position = np.hstack((P1_OV, P2_OV, P3_OV, P4_OV))
        
        
        
        # initialize Zk_kf 
        Zk_kf = np.matrix(np.zeros((28, 1)))

        # update v_0
        self.v_0 = self.v_0 + 0.01 * u_kf

        # calculation Zk based on the phase
        for i in range(4):
            if leg_status[i] == 0:
                self.estimated_contacts[0,i] = 1
            else:
                self.estimated_contacts[0,i] = 0

            Zk_kf[0+3*(i):3+3*(i), 0] = (-self.foot_position_O[:, i])
            Zk_kf[12+3*(i):15+3*(i),0] = (self.estimated_contacts[0,i] * (-foot_OV_position[:, i]))
            #Zk_kf[24+i-1,0] = (1-self.estimated_contacts[0,i]) * (self.Xk[2,0] + self.foot_position_O[2, i])

        # version 2
        for i in range(4):  # Python's range starts at 0 and goes to n-1
            a = 100
            factor = (1 + a * (1 - self.estimated_contacts[0,i]))
            R_noise[3*i:3*(i+1), 3*i:3*(i+1)] = 1 * R_noise[3*i:3*(i+1), 3*i:3*(i+1)]
            R_noise[12+3*i:15+3*i, 12+3*i:15+3*i] = factor * R_noise[12+3*i:15+3*i, 12+3*i:15+3*i]
            Q_noise[6+3*i:9+3*i, 6+3*i:9+3*i] = factor * Q_noise[6+3*i:9+3*i, 6+3*i:9+3*i]
            R_noise[24+i, 24+i] = factor* R_noise[24+i, 24+i]

        H_kf = np.matrix(np.zeros((28, 18)))

        for i in range(4):  
            H_kf[3*i:3*(i+1), 0:3] = np.matrix(np.eye(3))
            H_kf[3*i:3*(i+1), 6+3*i:9+3*i] = - np.matrix(np.eye(3))
            H_kf[3*i+12:3*(i+1)+12, 3:6] = np.matrix(np.eye(3))
            H_kf[24+i, 8+i*3] = 1

        # kalman filtercalculation
        self.Xk = self.A_kf @ self.Xk + self.B_kf @ u_kf
        Zk_ = H_kf @ self.Xk
        Pk_ = self.A_kf @ self.Pk @ self.A_kf.T + Q_noise
        SS = H_kf @ Pk_ @ H_kf.T + R_noise
        error_zk=Zk_kf-Zk_

        Serros_zk = np.linalg.solve(SS, error_zk)

        self.Xk = self.Xk + Pk_ @ H_kf.T @ Serros_zk
        SC = np.linalg.solve(SS, H_kf)
        self.Pk = (np.eye(18) - Pk_ @ H_kf.T @ SC) @ Pk_

        # update v_0 after calculation
        self.v_0 = self.Xk[3:6, 0]
        
        # record
        robot_position = self.Xk[0:3, 0]
        robot_velocity = self.Xk[3:6, 0]
        
        position = np.asarray(robot_position).reshape(3).tolist()
        velocity = np.asarray(robot_velocity).reshape(3).tolist()
        
        
        self.FL_pos = (np.array(self.foot_position_O[:, 0])).flatten()
        self.FR_pos = (np.array(self.foot_position_O[:, 1])).flatten()
        self.BL_pos = (np.array(self.foot_position_O[:, 2])).flatten()
        self.BR_pos = (np.array(self.foot_position_O[:, 3])).flatten()
        orientation = quaternions
        angular_speed = imu_ang_speed
        #print(position)     
       
        return position, velocity, orientation, angular_speed