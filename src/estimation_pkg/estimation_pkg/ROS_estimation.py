import rclpy
import csv
import time
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.msg import Buttons
from custom_interfaces.msg import LegReadings
from custom_interfaces.msg import LegStates
from custom_interfaces.msg import StateEstimation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import sys
import signal
from . StateEstimator_2 import StateEstimator
from sensor_msgs.msg import Imu

import csv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TopicRecorder(Node):

    def __init__(self):
        super().__init__('topic_recorder')
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
        self.states_received = [0,0,0,0]
        
        #Publisher
        self.publish_tf=True 
        self.odom_pub = self.create_publisher(Odometry, '/odometry/kalman', 10)
        
    

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.FL_tf_broadcaster = TransformBroadcaster(self)
            self.FR_tf_broadcaster = TransformBroadcaster(self)
            self.BL_tf_broadcaster = TransformBroadcaster(self)
            self.BR_tf_broadcaster = TransformBroadcaster(self)
        #SUSCRIBERS
        
        self.callback_group = ReentrantCallbackGroup()
        
       
        # Initialize timer for controlling publishing frequency
        self.publish_timer = self.create_timer(0.01, self.listener_callback, callback_group=self.callback_group)  # 100 Hz (0.01 seconds)
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
                '/imu_filtered',
                self.imu_state_callback,
                10, callback_group=self.callback_group)
        
        self.kalman_initialized = False
        self.Estimator = StateEstimator()
        
        self.CoM_pos = [0,0,0]
        self.CoM_vel = [0,0,0]
         # Create a callback group for asynchronous subscriptions
        
        self.CoM_orientation = [0,0,0,0]
        self.CoM_ang_speed = [0,0,0]
    
        self.latest_imu_data = None
        
    
    
    
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
        self.Estimator.initialize(self.FL_angles, self.FR_angles, self.BL_angles, self.BR_angles, self.latest_imu_data)
    
    
    def run_kalman(self):
        self.CoM_pos, self.CoM_vel, self.CoM_orientation, self.CoM_ang_speed = self.Estimator.Run(self.FL_angles, self.FR_angles, self.BL_angles, self.BR_angles, self.leg_status,self.latest_imu_data)
   
    
    def listener_callback(self):
       
        if self.kalman_initialized == False and all([self.FL_received, self.FR_received, self.BL_received, self.BR_received, self.imu_received]):
            self.init_kalman()
            self.kalman_initialized = True
            #print("Filter Started")
            pass
        elif self.kalman_initialized:
            current_time = time.time() - self.start_time  # Calculate elapsed time since node started
            
            state = [current_time]            
            
            self.run_kalman()
                         
            #Publish odometry message
            
            self.publish_odometry()             
                
                
                       
         
            
            
    def publish_odometry(self):
        current_time = self.get_clock().now().to_msg()

        # Create odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

         # Set position and orientation
         
        #point = Point(x=self.CoM_pos[0], y=self.CoM_pos[1], z=self.CoM_pos[2])
       # quaternion = Quaternion(x=self.CoM_orientation[0], y=self.CoM_orientation[1], z=self.CoM_orientation[2], w=self.CoM_orientation[3])
        
        odom.pose.pose.position.x = self.CoM_pos[0]
        odom.pose.pose.position.y = self.CoM_pos[1]
        odom.pose.pose.position.z = self.CoM_pos[2]
        # Set linear and angular velocity
        
        odom.pose.pose.orientation.x = self.CoM_orientation[0]
        odom.pose.pose.orientation.y = self.CoM_orientation[1]
        odom.pose.pose.orientation.z = self.CoM_orientation[2]
        odom.pose.pose.orientation.w = self.CoM_orientation[3]
         
    
         # Set linear and angular velocity
        odom.twist.twist.linear.x = self.CoM_vel[0]
        odom.twist.twist.linear.y = self.CoM_vel[1]
        odom.twist.twist.linear.z = self.CoM_vel[2]
        odom.twist.twist.angular.x = self.CoM_ang_speed[0]
        odom.twist.twist.angular.y = self.CoM_ang_speed[1]
        odom.twist.twist.angular.z = self.CoM_ang_speed[2]


        # Publish the odometry message
        self.odom_pub.publish(odom)

        
            
    def publish_odometry2(self):
        current_time = self.get_clock().now().to_msg()

        # Create odometry message
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'

         # Set position and orientation
         
        #point = Point(x=self.CoM_pos[0], y=self.CoM_pos[1], z=self.CoM_pos[2])
       # quaternion = Quaternion(x=self.CoM_orientation[0], y=self.CoM_orientation[1], z=self.CoM_orientation[2], w=self.CoM_orientation[3])
        
        odom.pose.pose.position.x = self.CoM_pos[0]
        odom.pose.pose.position.y = self.CoM_pos[1]
        odom.pose.pose.position.z = self.CoM_pos[2]
        # Set linear and angular velocity
        
        odom.pose.pose.orientation.x = self.CoM_orientation[0]
        odom.pose.pose.orientation.y = self.CoM_orientation[1]
        odom.pose.pose.orientation.z = self.CoM_orientation[2]
        odom.pose.pose.orientation.w = self.CoM_orientation[3]
         
    
         # Set linear and angular velocity
        odom.twist.twist.linear.x = self.CoM_vel[0]
        odom.twist.twist.linear.y = self.CoM_vel[1]
        odom.twist.twist.linear.z = self.CoM_vel[2]
        odom.twist.twist.angular.x = self.CoM_ang_speed[0]
        odom.twist.twist.angular.y = self.CoM_ang_speed[1]
        odom.twist.twist.angular.z = self.CoM_ang_speed[2]


        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Publish the transform if enabled
        self.publish_tf = False
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.CoM_pos[0]
            t.transform.translation.y = self.CoM_pos[1]
            t.transform.translation.z = self.CoM_pos[2]
            t.transform.rotation = Quaternion(x=self.CoM_orientation[0], y=self.CoM_orientation[1], z=self.CoM_orientation[2], w=self.CoM_orientation[3])
        

            self.tf_broadcaster.sendTransform(t)
            self.publish_leg_tf()
            #self.get_logger().info(str(self.Estimator.estimated_contacts))

        #self.get_logger().info('Publishing odometry and TF message')
    def publish_leg_tf(self):
        current_time = self.get_clock().now().to_msg()       
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'FL'
        t.transform.translation.x = self.Estimator.FL_pos[0] 
        t.transform.translation.y = self.Estimator.FL_pos[1] 
        t.transform.translation.z = self.Estimator.FL_pos[2]   
        self.FL_tf_broadcaster.sendTransform(t)
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'FR'
        t.transform.translation.x = self.Estimator.FR_pos[0] 
        t.transform.translation.y = self.Estimator.FR_pos[1] 
        t.transform.translation.z = self.Estimator.FR_pos[2]   
        self.FR_tf_broadcaster.sendTransform(t)
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'BL'
        t.transform.translation.x = self.Estimator.BL_pos[0] 
        t.transform.translation.y = self.Estimator.BL_pos[1] 
        t.transform.translation.z = self.Estimator.BL_pos[2]   
        self.BL_tf_broadcaster.sendTransform(t)
        
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'BR'
        t.transform.translation.x = self.Estimator.BR_pos[0] 
        t.transform.translation.y = self.Estimator.BR_pos[1] 
        t.transform.translation.z = self.Estimator.BR_pos[2]   
        self.BR_tf_broadcaster.sendTransform(t)
        
        
    def save_data(self):
        for row in self.write_data:
            self.csv_writer .writerow(row)
        print("SAVED DATA")
            
    
       
    def imu_state_callback(self, msg):         
  
        self.latest_imu_data = msg  
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
    
    executor = MultiThreadedExecutor(num_threads=1)
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
