import rclpy
from rclpy.node import Node
from custom_interfaces.msg import LegCommands
from . Crawl_Gait_Lib import Robot_Movement
from custom_interfaces.msg import LegStates  # Import your custom message type


class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')
        
        # Create publisher for LegCommands messages
        self.States_publisher_ = self.create_publisher(LegStates, 'leg_states', 10)
        self.FL_publisher_ = self.create_publisher(LegCommands, 'FL_leg_commands', 10)
        self.FR_publisher_ = self.create_publisher(LegCommands, 'FR_leg_commands', 10)
        self.BL_publisher_ = self.create_publisher(LegCommands, 'BL_leg_commands', 10)
        self.BR_publisher_ = self.create_publisher(LegCommands, 'BR_leg_commands', 10)
        # Initialize crawl gait variables
        self.elapsed_time = 0
        self.step = 0
        self.crawl_gait = Robot_Movement()
        self.freq = 100 # Hz
        self.dt = 1/self.freq
        self.states = [0,0,0,0]
        # Initialize timer for controlling publishing frequency
        self.publish_timer = self.create_timer(self.dt, self.publish_leg_commands)  # 100 Hz (0.01 seconds)
        
        self.FL_received = False
        self.FR_received = False
        self.BL_received = False
        self.BR_received = False
        
        self.imu_received = False
        self.states_received = False
        print("Start")
        
    def publish_leg_commands(self):
        msg = LegCommands()
        
        # Update crawl gait elapsed time
        self.crawl_gait.elapsed_time = self.elapsed_time

        # Determine the current step of the crawl gait
        if self.step == 0:
            if self.crawl_gait.start() == 1:
                # Prompt user to input key command
                #print("Press any key to continue...")
                #input()  # Wait for user input
                self.elapsed_time = 0
                self.crawl_gait.elapsed_time = 0
                self.step = 1
        elif self.step == 1:        
            if self.crawl_gait.Tilt(1) == 1:
                self.elapsed_time = 0
                self.step = 2
        elif self.step == 2:        
            if self.crawl_gait.Swing_Leg("BL") == 1:
                self.elapsed_time = 0
                self.step = 3
        
        elif self.step == 3:        
            if self.crawl_gait.Swing_Leg("FL") == 1:
                self.elapsed_time = 0
                self.step = 4
            
        elif self.step == 4:        
            if self.crawl_gait.Tilt(-1) == 1:
                self.elapsed_time = 0
                self.step = 5
                
        
        elif self.step == 5:        
            if self.crawl_gait.Swing_Leg("BR") == 1:
                self.elapsed_time = 0
                self.step = 6
        
        elif self.step == 6:        
            if self.crawl_gait.Swing_Leg("FR") == 1:
                self.elapsed_time = 0
                #print("GAIT FINISHED press any key to restart...")
                #input()  # Wait for user input
                self.step = 1
                
        elif self.step == 7:        
            if self.crawl_gait.Swing_Leg_3D("FL") == 1:
                self.elapsed_time = 0          
                self.step = 0
        
        
        # Increment elapsed time
        self.elapsed_time += self.dt
        
        # Example: Retrieve joint angles from crawl gait
        leg_angles = self.crawl_gait.get_joint_position()
        
        # Fill in LegCommands message with actual values
        msg.control_mode = 4  # 4 - Position Control
        msg.joint_angles = leg_angles[0]  # Actual joint angles from crawl gait
        msg.joint_speed = [self.crawl_gait.speed, self.crawl_gait.speed, self.crawl_gait.speed]  # Example joint speeds
        msg.joint_torques = [0.0, 0.0, 0.0]  # Example joint torques
        #print(leg_angles[0])
        # Publish the message
        self.FL_publisher_.publish(msg)
        msg.joint_angles = leg_angles[1]  # Actual joint angles from crawl gait
        self.FR_publisher_.publish(msg)
        msg.joint_angles = leg_angles[2]  # Actual joint angles from crawl gait
        self.BL_publisher_.publish(msg)
        msg.joint_angles = leg_angles[3]  # Actual joint angles from crawl gait
        self.BR_publisher_.publish(msg)
        
        msg_states = LegStates()
        msg_states.leg_states = self.crawl_gait.states
        #print(msg_states)
        self.States_publisher_.publish(msg_states)
        
def main(args=None):
    rclpy.init(args=args)
    leg_controller = LegController()
    rclpy.spin(leg_controller)
    leg_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
