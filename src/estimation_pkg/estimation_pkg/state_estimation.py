import rclpy
import csv
import time
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.msg import Buttons
from custom_interfaces.msg import LegReadings
from custom_interfaces.msg import LegStates
from custom_interfaces.msg import StateEstimation
import sys
import signal
from . StateEstimator_Py import StateEstimator
from sensor_msgs.msg import Imu

import csv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TopicRecorder(Node):

    def __init__(self):
        super().__init__('topic_recorder')
        self.data_file = '/home/pi/ros2_ws/src/estimation_pkg/estimation_pkg/estimation_data.csv'
        self.csv_file = None
        self.csv_writer = None
        self.start_time = time.time()  # Record the time when the node startse
        self.latest_imu_state = None  # Variable to store the latest LED state
        self.latest_imu_quaternion = None
        self.latest_imu_acceleration = None
        self.latest_imu_speed = None
        self.latest_state = 0
        self.leg_status = [0,0,0,0]
        self.write_data = []
        
        #FL 
        self.FL_angles = [None, None, None]       
        self.FL_speeds = [None, None, None]
        
        #FR
        self.FR_angles = [None, None, None]       
        self.FR_speeds = [None, None, None]
               
        #BL        
        self.BL_angles = [None, None, None]       
        self.BL_speeds = [None, None, None]
        
        #BR      
        self.BR_angles = [None, None, None]       
        self.BR_speeds = [None, None, None]
        
        self.FL_received = False
        self.FR_received = False
        self.BL_received = False
        self.BR_received = False
        
        self.imu_received = False
        self.states_received = True
        
        
        self.estimation_publisher_ = self.create_publisher(StateEstimation, 'state_estimation', 10)
        
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.callback_group2 = ReentrantCallbackGroup()
        # Initialize timer for controlling publishing frequency
        self.publish_timer = self.create_timer(0.01, self.listener_callback, callback_group=self.callback_group2)  # 100 Hz (0.01 seconds)
        self.key_subscription = self.create_subscription(
            String,
            'key_input',
            self.key_callback,
            10, callback_group=self.callback_group)
        
        self.FL_sup = self.create_subscription(
            LegReadings,
            '/leg_readings_FL',
            self.FL_readings_callback,
            10, callback_group=self.callback_group)
        self.FR_sup = self.create_subscription(
            LegReadings,
            '/leg_readings_FR',
            self.FR_readings_callback,
            10, callback_group=self.callback_group)
        self.BL_sup = self.create_subscription(
            LegReadings,
            '/leg_readings_BL',
            self.BL_readings_callback,
            10, callback_group=self.callback_group)
        self.BR_sup = self.create_subscription(
            LegReadings,
            '/leg_readings_BR',
            self.BR_readings_callback,
            10, callback_group=self.callback_group)
        self.St_sub = self.create_subscription(
            LegStates,
            '/leg_states',
            self.leg_states_callback,
            10, callback_group=self.callback_group)
        
        self.imu_sub = self.create_subscription(
                Imu,  # Use appropriate message type for the /led_state topic
                '/bno055/imu',
                self.imu_state_callback,
                10, callback_group=self.callback_group)
        
        self.kalman_initialized = False
        self.Estimator = StateEstimator()
        self.position_history = []
        self.velocity_history = []
        self.CoM_pos = []
        self.CoM_vel = []
         # Create a callback group for asynchronous subscriptions
        
        
    
        
        
    
    
    
    def leg_states_callback(self, msg):
        self.leg_status = msg.leg_states
        self.states_received = True
            
    def FL_readings_callback(self, msg):
        self.FL_angles = [msg.shoulder_joint_angle, msg.hip_joint_angle, msg.knee_joint_angle]       
        self.FL_speeds = [msg.shoulder_joint_speed, msg.hip_joint_speed, msg.knee_joint_speed]
        self.FL_received = True
       
    def FR_readings_callback(self, msg):
        self.FR_angles = [msg.shoulder_joint_angle, msg.hip_joint_angle, msg.knee_joint_angle]       
        self.FR_speeds = [msg.shoulder_joint_speed, msg.hip_joint_speed, msg.knee_joint_speed]
        self.FR_received = True

    def BL_readings_callback(self, msg):
        self.BL_angles = [msg.shoulder_joint_angle, msg.hip_joint_angle, msg.knee_joint_angle]       
        self.BL_speeds = [msg.shoulder_joint_speed, msg.hip_joint_speed, msg.knee_joint_speed]
        self.BL_received = True

    def BR_readings_callback(self, msg):
        self.BR_angles = [msg.shoulder_joint_angle, msg.hip_joint_angle, msg.knee_joint_angle]       
        self.BR_speeds = [msg.shoulder_joint_speed, msg.hip_joint_speed, msg.knee_joint_speed]
        self.BR_received = True
    
    def init_kalman(self):
        self.Estimator.initialize(self.FL_angles, self.FR_angles, self.BL_angles, self.BR_angles, self.latest_imu_quaternion, self.latest_imu_acceleration, self.latest_imu_speed)
    
    
    def run_kalman(self):
        self.CoM_pos, self.CoM_vel = self.Estimator.Run(self.FL_angles, self.FR_angles, self.BL_angles, self.BR_angles, self.leg_status,self.latest_imu_quaternion, self.latest_imu_acceleration, self.latest_imu_speed)
    
    
    def listener_callback(self):
        if self.csv_writer is None:
            self.csv_file = open(self.data_file, 'w')
            self.csv_writer = csv.writer(self.csv_file, delimiter=',')
            # Write header
            time_header = ['Timestamp']
            position_header = ["X", "Y", "Z"]
            velocity_header = ["Vx", "Vy", "Vz"]
            
            header = time_header + position_header + velocity_header
            self.csv_writer.writerow(header)
            
            #time.sleep(5)
        #print(self.FL_received, self.FR_received, self.BL_received, self.BR_received, self.imu_received, self.states_received)
        if self.kalman_initialized == False and all([self.FL_received, self.FR_received, self.BL_received, self.BR_received, self.imu_received, self.states_received]):
            self.init_kalman()
            self.kalman_initialized = True
            print("Filter Started")
        if self.kalman_initialized:
            current_time = time.time() - self.start_time  # Calculate elapsed time since node started
            
            state = [current_time]
            
            try:
                self.run_kalman()
                row = state + self.CoM_pos + self.CoM_vel
                #print(self.CoM_pos) 
            except:
                row = state
                
            self.write_data.append(row)
            
            
            #self.csv_writer .writerow(row)
    
    def save_data(self):
        for row in self.write_data:
            self.csv_writer .writerow(row)
        print("SAVED DATA")
            
    
       
    def imu_state_callback(self, msg):
        # Callback function to update the latest LED state
        
        quaternion = msg.orientation
        ang_speed = msg.angular_velocity
        lin_acc = msg.linear_acceleration
        self.latest_imu_quaternion = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        self.latest_imu_acceleration = [lin_acc.x, lin_acc.y, lin_acc.z]
        self.latest_imu_speed = [ang_speed.x, ang_speed.y, ang_speed.z]
        self.imu_received = True
        
        
    def state_callback(self, msg):
        latest_state = msg.state
    
    def key_callback(self, msg):
        self.get_logger().info('Received key: %s' % msg.data)
        rclpy.shutdown()
        sys.exit()
        
def main(args=None):
    rclpy.init(args=args)
    topic_recorder = TopicRecorder()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(topic_recorder)
    try:
        executor.spin()
    except KeyboardInterrupt:
        topic_recorder.save_data()
        print('Keyboard interrupt, shutting down...')
    finally:
        topic_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
