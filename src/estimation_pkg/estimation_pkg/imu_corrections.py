import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import butter, filtfilt, lfilter, lfilter_zi
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.signal import lfilter
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class IMUCorrectionNode(Node):
    def __init__(self):
        super().__init__('imu_correction_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu_rotated',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu_filtered', 10)
        self.max_twist = np.radians(45) #deg
        self.orientation_buffer = None
        self.initialized = False

        self.samples = 0
        self.max_acc = 0.5 #m/s
        self.max_vel = 10  #rad/s
    def saturate(self,value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value
        
    def initialize(self, imu_data):
        # Filter design
        Fs = 100  # sample frequency
        Fc = 5  # cutoff frequency
        n = 2     # order
        # Butterworth low
        self.b, self.a = butter(n, Fc/(Fs/2), btype='low')
        Fs = 100  # sample frequency
        Fc = 5   # cutoff frequency
        n = 2     # order
        # Butterworth low
        self.b_orientation, self.a_orientation = butter(n, Fc/(Fs/2), btype='low')
        
        acc = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z]
        ang_speed = [imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z]
        
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        
        # Quaternions to Euler
        eulerAngles = np.zeros((1, 3))
        euler = euler_from_quaternion(quaternions)
        eulerAngles[0, :] = euler
        
        # Adjust the angles
        eulerAngles = eulerAngles[0]
        eulerAngles[0] += np.radians(180)
        
        self.orientation_buffer = eulerAngles
        # Initial signal
        self.zi_r = lfilter_zi(self.b_orientation, self.a_orientation) * eulerAngles[0]
        self.zi_p = lfilter_zi(self.b_orientation, self.a_orientation) * eulerAngles[1]
        self.zi_y = lfilter_zi(self.b_orientation, self.a_orientation) * eulerAngles[2]

        self.a_imu_1 = lfilter_zi(self.b, self.a) * acc[0]
        self.a_imu_2 = lfilter_zi(self.b, self.a) * acc[1]
        self.a_imu_3 = lfilter_zi(self.b, self.a) * acc[2]

        self.w_imu_1 = lfilter_zi(self.b, self.a) * ang_speed[0]
        self.w_imu_2 = lfilter_zi(self.b, self.a) * ang_speed[1]
        self.w_imu_3 = lfilter_zi(self.b, self.a) * ang_speed[2]
        self.initialized = True
        
    def correct_imu_data(self, imu_data):
        acc = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z]
        ang_speed = [imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z]
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]

        imu_acc = np.array(acc)
        imu_ang_speed = np.array(ang_speed)

        # Quaternions to Euler    
        r_, p_, y_ = euler_from_quaternion(quaternions)
        
        r_ += np.radians(180)
        if self.samples >= 50:
            #print(round(y_ * 180 / np.pi, 1), round(p_ * 180 / np.pi, 1), round(r_ * 180 / np.pi, 1))
            self.samples = 0
        else:
            self.samples += 1
        diff_r = abs(r_ - self.orientation_buffer[0])
        diff_p = abs(p_ - self.orientation_buffer[1])
        diff_y = abs(y_ - self.orientation_buffer[2])
        
        if diff_r > self.max_twist or diff_p > self.max_twist or diff_y > self.max_twist:
            r_ = self.orientation_buffer[0]
            p_ = self.orientation_buffer[1]
            y_ = self.orientation_buffer[2]
        
        else:
            self.orientation_buffer = [r_, p_, y_]
            
        r_, self.zi_y = lfilter(self.b_orientation, self.a_orientation, [r_], zi=self.zi_y)
        p_, self.zi_p = lfilter(self.b_orientation, self.a_orientation, [p_], zi=self.zi_p)
        y_, self.zi_r = lfilter(self.b_orientation, self.a_orientation, [y_], zi=self.zi_r)

        y_, p_, r_ = np.float64(y_[0]), np.float64(p_[0]), np.float64(r_[0])
        
        
        
        #print(diff_r, diff_p, diff_y)
        
        imu_acc[0] = self.saturate(imu_acc[0], -self.max_acc, self.max_acc)
        imu_acc[1] = self.saturate(imu_acc[1], -self.max_acc, self.max_acc)
        imu_acc[2] = self.saturate(imu_acc[2], -self.max_acc, self.max_acc)
        
        a_imu_x, self.a_imu_1 = lfilter(self.b, self.a, [imu_acc[0]], zi=self.a_imu_1)
        a_imu_y, self.a_imu_2 = lfilter(self.b, self.a, [imu_acc[1]], zi=self.a_imu_2)
        a_imu_z, self.a_imu_3 = lfilter(self.b, self.a, [imu_acc[2]], zi=self.a_imu_3)
        a_imu_x, a_imu_y, a_imu_z = np.float64(a_imu_x[0]), np.float64(a_imu_y[0]), np.float64(a_imu_z[0])
        
        
       
        
        
        a_imu = np.matrix([[a_imu_x], [a_imu_y], [a_imu_z]])

        w_imu_x, self.w_imu_1 = lfilter(self.b, self.a, [imu_ang_speed[0]], zi=self.w_imu_1)
        w_imu_y, self.w_imu_2 = lfilter(self.b, self.a, [imu_ang_speed[1]], zi=self.w_imu_2)
        w_imu_z, self.w_imu_3 = lfilter(self.b, self.a, [imu_ang_speed[2]], zi=self.w_imu_3)
        w_imu_x, w_imu_y, w_imu_z = np.float64(w_imu_x[0]), np.float64(w_imu_y[0]), np.float64(w_imu_z[0])
        w_imu = np.matrix([[w_imu_x], [w_imu_y], [w_imu_z]])

        # Update the IMU message
        imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z = a_imu_x, a_imu_y, a_imu_z
        imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z = w_imu_x, w_imu_y, w_imu_z

        corrected_quaternion = quaternion_from_euler(r_, p_, y_)
        imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w = corrected_quaternion

        self.publisher.publish(imu_data)
        
    def imu_callback(self, msg):
        if self.initialized:
            corrected_msg = self.correct_imu_data(msg)         
        else:
            self.initialize(msg)
            print("Initialized")

        
    

    def R1(self, y, p, r):
        # Compute rotation matrix R1 from yaw, pitch, roll
        R_yaw = np.array([
            [np.cos(y), -np.sin(y), 0],
            [np.sin(y), np.cos(y), 0],
            [0, 0, 1]
        ])
        R_pitch = np.array([
            [np.cos(p), 0, np.sin(p)],
            [0, 1, 0],
            [-np.sin(p), 0, np.cos(p)]
        ])
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(r), -np.sin(r)],
            [0, np.sin(r), np.cos(r)]
        ])
        return R_yaw @ R_pitch @ R_roll

def main(args=None):
    rclpy.init(args=args)
    node = IMUCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
