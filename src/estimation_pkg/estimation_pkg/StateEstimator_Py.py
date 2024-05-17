import numpy as np
from scipy.signal import butter, filtfilt, lfilter, lfilter_zi
from scipy.spatial.transform import Rotation as Rot
from . forward_kinematics import ForwardK1
from . forward_kinematics import CrossMatrix
from . forward_kinematics import R1

class StateEstimator:
    def __init__(self):
        pass
    
    def initialize(self, FL, FR, BL, BR, quaternions,acc, ang_speed):
        
        FL_angles =  np.array(FL)
        FR_angles =  np.array(FR)
        BL_angles =  np.array(BL)
        BR_angles =  np.array(BR)
        imu_acc = np.array(acc)
        imu_ang_speed = np.array(ang_speed)
        # Quaternions to euler  
        qt = np.array(quaternions)
        # Initialize an array to store Euler angles
        eulerAngles = np.zeros((1, 3))
        # Reorder the quaternion
        quat_reordered = qt[[1, 2, 3, 0]]
        # Create a rotation object from the reordered quaternion
        r = Rot.from_quat(quat_reordered)
        # Convert the rotation to Euler angles, ZYX order
        euler = r.as_euler('ZYX')
        # Store the computed Euler angles
        eulerAngles[0, :] = euler
        # Adjust the angles
        eulerAngles[:, 0] += np.radians(90)  # Adjusting the first angle
        eulerAngles[:, 2] -= np.radians(90) 
        eulerAngles = eulerAngles[0]
        # modify  imu_acc[1]
        imu_acc[1] = imu_acc[1] - 0.4

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
        self.dt = 0.01

        # A_kf
        self.A_kf = np.matrix(np.eye(18))
        self.A_kf[0:3, 3:6] = self.dt * np.eye(3)

        # B_kf
        self.B_kf = np.matrix(np.zeros((18, 3)))
        self.B_kf[3:6, 0:3] = self.dt * np.eye(3)

        # g= 0 0 0 because g is already balestimated_contactsanced by other code
        self.ag = np.matrix([[0], [0], [0]])

      

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

        # coordinates relationships

        # imu to body
        self.imu_R_body = np.matrix([[0, -1, 0],
                            [1, 0, 0],
                            [0, 0, 1]])

        # body to imu which is fixed
        self.body_R_imu = self.imu_R_body.T

        # inital position body to imu
        # initial body coordi is same as world coordi
        R_0 = self.body_R_imu

        # imu0 is same as world --> world to imu = imu0 to imu
        self.imu0_R_imu = R_0

        # filter design
        Fs = 100  # sample frequency
        Fc = 10   # cutoff frequency
        n = 3      # order


        # Butterworth low
        self.b, self.a = butter(n, Fc/(Fs/2), btype='low')
        
        # initial signal
        self.zi_y = lfilter_zi(self.b, self.a) * eulerAngles[0]
        self.zi_p = lfilter_zi(self.b, self.a) * eulerAngles[1]
        self.zi_r = lfilter_zi(self.b, self.a) * eulerAngles[2]


        self.a_imu_1 = lfilter_zi(self.b, self.a) * imu_acc[0]
        self.a_imu_2 = lfilter_zi(self.b, self.a) * imu_acc[1]
        self.a_imu_3 = lfilter_zi(self.b, self.a) * imu_acc[2]

        self.w_imu_1 = lfilter_zi(self.b, self.a) * imu_ang_speed[0]
        self.w_imu_2 = lfilter_zi(self.b, self.a) * imu_ang_speed[1]
        self.w_imu_3 = lfilter_zi(self.b, self.a) * imu_ang_speed[2]
        
    def Run(self, FL, FR, BL, BR, leg_status, quaternions,acc, ang_speed):
        
        
        
        FL_angles =  np.array(FL)
        FR_angles =  np.array(FR)
        BL_angles =  np.array(BL)
        BR_angles =  np.array(BR)
        imu_acc = np.array(acc)
        imu_ang_speed = np.array(ang_speed)
        #Quaternions to euler  
        qt = np.array(quaternions)
        # Initialize an array to store Euler angles
        eulerAngles = np.zeros((1, 3))
        # Reorder the quaternion
        quat_reordered = qt[[1, 2, 3, 0]]
        # Create a rotation object from the reordered quaternion
        r = Rot.from_quat(quat_reordered)
        # Convert the rotation to Euler angles, ZYX order
        euler = r.as_euler('ZYX')
        # Store the computed Euler angles
        eulerAngles[0, :] = euler
        # Adjust the angles
        eulerAngles[:, 0] += np.radians(90)  # Adjusting the first angle
        eulerAngles[:, 2] -= np.radians(90) 
        eulerAngles = eulerAngles[0]
        imu_acc[1] = imu_acc[1] - 0.4
        
        
        #KALMAN FILTER LOOP
        # yaw pitch roll
        y_, p_, r_ = eulerAngles[0], eulerAngles[1], eulerAngles[2]

        y_, self.zi_y = lfilter(self.b, self.a, [eulerAngles[0]], zi=self.zi_y)
        p_, self.zi_p = lfilter(self.b, self.a, [eulerAngles[1]], zi=self.zi_p)
        r_, self.zi_r = lfilter(self.b, self.a, [eulerAngles[2]], zi=self.zi_r)

        y_ = np.float64(y_[0])
        p_ = np.float64(p_[0])
        r_ = np.float64(r_[0])

        # acc_imu
        a_imu_x, self.a_imu_1 = lfilter(self.b, self.a, [imu_acc[0]], zi=self.a_imu_1)
        a_imu_y, self.a_imu_2 = lfilter(self.b, self.a, [imu_acc[1]], zi=self.a_imu_2)
        a_imu_z, self.a_imu_3 = lfilter(self.b, self.a, [imu_acc[2]], zi=self.a_imu_3)

        a_imu_x = np.float64(a_imu_x[0])
        a_imu_y = np.float64(a_imu_y[0])
        a_imu_z = np.float64(a_imu_z[0])

        a_imu = np.matrix([[a_imu_x], 
                        [a_imu_y],
                        [a_imu_z]])

        # joint angle

        pss = np.vstack((FL_angles, FR_angles, BL_angles, BR_angles))
        pss = np.asmatrix(pss)

        # wb
        w_imu_x, self.w_imu_1 = lfilter(self.b, self.a, [imu_ang_speed[0]], zi=self.w_imu_1)
        w_imu_y, self.w_imu_2 = lfilter(self.b, self.a, [imu_ang_speed[1]], zi=self.w_imu_2)
        w_imu_z, self.w_imu_3 = lfilter(self.b, self.a, [imu_ang_speed[2]], zi=self.w_imu_3)

        w_imu_x = np.float64(w_imu_x[0])
        w_imu_y = np.float64(w_imu_y[0])
        w_imu_z = np.float64(w_imu_z[0])

        w_imu = np.matrix([[w_imu_x], [w_imu_y], [w_imu_z]])

        # O_R_B
        R_t = R1(y_, p_, r_)
        R = self.imu0_R_imu @ R_t @ self.imu_R_body

        rotY = np.matrix([
            [np.cos(y_), -np.sin(y_), 0],
            [np.sin(y_), np.cos(y_), 0],
            [0, 0, 1]
        ])

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
        ab = self.body_R_imu @ a_imu

        #  omega_body
        wb = self.body_R_imu @ w_imu

        wb_cm = CrossMatrix(wb)

        u_kf = R @ ab + self.ag

        # Initialize Q_noise and R_noise matrices
        Q_noise = np.matrix(np.zeros((18, 18)))

        # version 2
        # most related to Q_noist 1e5 
        Q_noise[0:3, 0:3] =  self.PROCESS_NOISE * self.dt * np.matrix(np.eye(3)) / 20
        #Q_noise[2, 2] = 0.01 * self.dt / 20
        Q_noise[3:6, 3:6] =  9.8 * self.PROCESS_NOISE * self.dt * np.matrix(np.eye(3)) / 20
        R_noise =  self.PROCESS_NOISE * np.matrix(np.eye(28))

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
            Zk_kf[12+3*(i):15+3*(i),0] = (1 - self.estimated_contacts[0,i]) * self.v_0 + (self.estimated_contacts[0,i] * (-foot_OV_position[:, i]))
            Zk_kf[24+i-1,0] = (1 - self.estimated_contacts[0,i]) * (self.Xk[2,0] + self.foot_position_O[2, i])

        # version 2
        for i in range(4):  # Python's range starts at 0 and goes to n-1
            a = 100
            factor = (1 + a * (1 - self.estimated_contacts[0,i]))
            R_noise[3*i:3*(i+1), 3*i:3*(i+1)] = factor * np.matrix(np.eye(3))
            R_noise[12+3*i:15+3*i, 12+3*i:15+3*i] = factor * np.matrix(np.eye(3))
            Q_noise[6+3*i:9+3*i, 6+3*i:9+3*i] = factor * self.dt * np.matrix(np.eye(3))
            R_noise[24+i, 24+i] = factor

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


        return position, velocity
        
    
    