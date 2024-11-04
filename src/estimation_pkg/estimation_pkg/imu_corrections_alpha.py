import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.signal import butter, filtfilt, lfilter, lfilter_zi
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.signal import lfilter
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header

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
        self.yaw_offset = None
        self.alpha_acc = 0.01
        self.alpha_ang_vel = 0.15
        self.prev_acc = None
        self.prev_ang_vel = None
        self.max_acc = 1.0 #m/s
        self.max_vel = 1.0  #rad/s
    
    def low_pass_filter(self,new_value, prev_value, alpha):
        if prev_value is None:            
            # If no previous value, return the new value as the filtered value
            return new_value
        
        return alpha * new_value + (1 - alpha) * prev_value
    def saturate(self,value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value
        
    def initialize(self, imu_data):
       
        
        acc = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
        ang_speed = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]        
        # Quaternions to Euler
        r_, p_, y_ = euler_from_quaternion(quaternions)
        r_ += np.radians(180)
        
        self.yaw_offset = y_
        self.orientation_buffer = [r_, p_, 0]
        self.prev_acc = acc
        self.prev_ang_vel = ang_speed

        R = self.R1(r_,p_,y_)
       
        a_imu = np.matrix([[acc[0]], 
                        [acc[1]],
                        [acc[2]]])
        u_kf = R @ a_imu
        # g= 0 0 0 because g is already balestimated_contactsanced by other code

       

        acc_offset = (np.array(u_kf)).flatten()

        self.get_logger().info("Acc Offset: [%.2f, %.2f, %.2f]" %
                              (acc_offset[0], 
                                acc_offset[1],
                                acc_offset[2]))
        
        self.initialized = True


      

        
    def correct_imu_data(self, imu_data):
        acc = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
        ang_speed = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        
        quaternions = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
        
        imu_acc = acc
        imu_ang_speed = ang_speed
        r_, p_, y_ = euler_from_quaternion(quaternions)
        r_ += np.radians(180)
        y_ -= self.yaw_offset

        diff_r = abs(r_ - self.orientation_buffer[0])
        diff_p = abs(p_ - self.orientation_buffer[1])
        diff_y = abs(y_ - self.orientation_buffer[2])
        
        #print(diff_r, diff_p, diff_y)
        if self.orientation_buffer is None:
            self.orientation_buffer = [r_, p_, y_]
            
        elif diff_r > self.max_twist or diff_p > self.max_twist or diff_y > self.max_twist:
            r_ = self.orientation_buffer[0]
            p_ = self.orientation_buffer[1]
            y_ = self.orientation_buffer[2]
        
        else:
            self.orientation_buffer = [r_, p_, y_]          
               
        
        
        a_imu_x = self.low_pass_filter(imu_acc[0], self.prev_acc[0], self.alpha_acc)
        a_imu_y = self.low_pass_filter(imu_acc[1], self.prev_acc[1], self.alpha_acc)
        a_imu_z = self.low_pass_filter(imu_acc[2], self.prev_acc[2], self.alpha_acc)
        
        a_imu_x = self.saturate(a_imu_x, -self.max_acc, self.max_acc)
        a_imu_y = self.saturate(a_imu_y, -self.max_acc, self.max_acc)
        a_imu_z = self.saturate(a_imu_z, -self.max_acc, self.max_acc)
              
                
        # Apply low-pass filter to angular velocity
        w_imu_x = self.low_pass_filter(imu_ang_speed[0], self.prev_ang_vel[0], self.alpha_ang_vel)
        w_imu_y = self.low_pass_filter(imu_ang_speed[1], self.prev_ang_vel[1], self.alpha_ang_vel)
        w_imu_z = self.low_pass_filter(imu_ang_speed[2], self.prev_ang_vel[2], self.alpha_ang_vel)        
       
              
        w_imu_x = self.saturate(w_imu_x, -self.max_vel, self.max_vel)
        w_imu_y = self.saturate(w_imu_y, -self.max_vel, self.max_vel)
        w_imu_z = self.saturate(w_imu_z, -self.max_vel, self.max_vel)
              



        try:
            imu_data = Imu()
            imu_data.header = Header()
            imu_data.header.stamp = self.get_clock().now().to_msg()
             # Update the IMU message
            imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z = a_imu_x, a_imu_y, a_imu_z
            imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z = w_imu_x, w_imu_y, w_imu_z

            corrected_quaternion = quaternion_from_euler(r_, p_, y_)
            imu_data.orientation.x = corrected_quaternion[0]
            imu_data.orientation.y = corrected_quaternion[1]
            imu_data.orientation.z = corrected_quaternion[2]
            imu_data.orientation.w = corrected_quaternion[3]
           

            self.publisher.publish(imu_data)
        except:
            self.get_logger().info("Quat: [%.2f, %.2f, %.2f]" %
                              (r_, 
                                p_,
                                y_))
            self.get_logger().info("Angular Velocity: [%.2f, %.2f, %.2f]" %
                                  (w_imu_x, 
                                    w_imu_y, 
                                    w_imu_z))
            self.get_logger().info("Linear Acceleration: [%.2f, %.2f, %.2f]" % 
                                   (a_imu_x, 
                                    a_imu_y, 
                                    a_imu_z))
            
        
   
        
    def imu_callback(self, msg):
        if self.initialized:
            corrected_msg = self.correct_imu_data(msg)         
        else:
            self.initialize(msg)
            print("Initialized")

        
    

    def R1(self, alpha, beta, gamma):
        t2 = np.cos(alpha)
        t3 = np.cos(beta)
        t4 = np.cos(gamma)
        t5 = np.sin(alpha)
        t6 = np.sin(beta)
        t7 = np.sin(gamma)
        w = np.matrix([
            [t2*t3, t3*t5, -t6],
            [-t4*t5 + t2*t6*t7, t2*t4 + t5*t6*t7, t3*t7],
            [t5*t7 + t2*t4*t6, -t2*t7 + t4*t5*t6, t3*t4]
        ])
        return w

def main(args=None):
    rclpy.init(args=args)
    node = IMUCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
