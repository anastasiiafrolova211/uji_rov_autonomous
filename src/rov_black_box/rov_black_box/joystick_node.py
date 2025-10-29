import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong
from time import sleep

class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        
        self.get_logger().info('Starting BlueROV joystick')
        self.subscription = self.create_subscription(Joy, '/bluerov2/joy', self.joyCallback, 10)

        # initialize variables
        self.set_mode = [True, False, False]
        self.arming = False

        # used pins
        # pins:
        # 1-8 are thrusters
        # 9 
        # 10
        # 11 light
        # 12
        # 13 gripper
        # 14
        # 15 camera tilting - need to make sure with this one
        # 16

        self.light_pin = 11.0
        self.gripper_pin = 13.0
        self.camera_servo_pin = 15.0

        #light values 
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        
        # camera servo
        self.tilt = 1450.0
        self.tilt_int = 1450.0 # for keeping neutral horizontal position
        self.servo_min = 1100.0
        self.servo_max = 1850.0

        # gripper
        self.gripper = 1100.0
        self.gripper_min = 1100.0
        self.gripper_max = 1580.0

        ## Intail test for the system
        self.run_initialization_test = True
        if self.run_initialization_test:
            self.initialization_test()


# TODO do i need to add init test for gripper?

    def initialization_test(self):
        """
        Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem.
        """
        self.get_logger().info("Testing light and camera servo...")
        
        # Flash the light
        self.light = self.light_max
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        
        # Move the camera servo to max and min
        self.tilt = self.tilt_int
        self.send_servo_command(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_max
        self.send_servo_command(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_min
        self.send_servo_command(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_int  # Reset camera tilt to neutral
        self.send_servo_command(self.camera_servo_pin, self.tilt)


    def send_servo_command(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'bluerov2/cmd/command') # updated topic name
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return
        
        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request = CommandLong.Request()
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0    

        sleep(3)
        future = client.call_async(request)

        # Check the result
        if future.result() is not None:
            self.get_logger().info('Change Completed')
            self.get_logger().info(f'Sent servo command pin {pin_number}, value {value}')

        else:
            self.get_logger().error('Failed to preform the change ')


    def armDisarm(self, armed):
        '''
        Arms or disarms the vehicle motors using MAVROS command 400.
        '''
        cli = self.create_client(CommandLong, 'cmd/command')  # Create MAVROS service client
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)  # Wait for service to be available
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        # Create request object for arming/disarming
        req = CommandLong.Request()
        req.broadcast = False   # Command is not broadcasted
        req.command = 400       # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0    # No confirmation required
        req.param1 = 1.0 if armed else 0.0  # 1.0 = Arm, 0.0 = Disarm
        req.param2 = 0.0  
        req.param3 = 0.0  
        req.param4 = 0.0  
        req.param5 = 0.0  
        req.param6 = 0.0  
        req.param7 = 0.0 
        
        self.get_logger().info("Sending command...")
        resp = cli.call_async(req)  # Send command asynchronously
        
        # Log the result
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")

    def joyCallback(self, data):
        ''' 
        Map the Joystick buttons according the bluerov configuration as descriped at
        https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg
        **Note: the lights are set to be in RT and LT button instead of the cross buttons
        '''
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button
        btn_camera_servo_up = data.buttons[4] # LB button 
        btn_camera_servo_down = data.buttons[5] # RB button 
        btn_camera_rest = data.buttons[9] # R3 button 

        btn_light_down = data.axes[2] # LT button
        btn_light_up = data.axes[5] # RT button

        btn_gripper = data.axes[6]  # left/right arrow buttons


        # Disarming when Back button is pressed
        if btn_disarm == 1 and self.arming:
            self.arming = False
            self.armDisarm(False)
        # Arming when Start button is pressed
        if btn_arm == 1 and not self.arming:
            self.arming = True
            self.armDisarm(True)

        # Switch manual, auto anset_moded correction mode
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.get_logger().info("Mode manual")
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.get_logger().info("Mode automatic")
        elif btn_corrected_mode and not self.set_mode[2]:
            self.set_mode = [False, False, True]
            self.get_logger().info("Mode correction")

        #### Control light intensity####
        if (btn_light_up == -1 and self.light < self.light_max):
            self.light = min(self.light + 100.0, self.light_max)
            self.send_servo_command(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")
            
        elif (btn_light_down == -1 and self.light > self.light_min):
            self.light = max(self.light_min,self.light - 100)
            self.send_servo_command(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")

        ### Control Camera tilt angle ###
        if (btn_camera_servo_up and not btn_camera_servo_down and self.tilt < self.servo_max):
            self.tilt = min(self.servo_max, self.tilt + 100)
            self.send_servo_command(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_servo_down and self.tilt > self. servo_min):
            self.tilt = max(self.servo_min, self.tilt - 100)
            self.send_servo_command(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_rest):
            self.tilt = self.tilt_int
            self.send_servo_command(self.camera_servo_pin,self.tilt)
            self.get_logger().info(f"Camera tilt has been reseted")



        # Gripper control buttons
        if btn_gripper == 1.0 and self.gripper < self.gripper_max:
            # right -> open gripper
            self.gripper = min(self.gripper + 50, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening. PWM: {self.gripper}")
        elif btn_gripper == -1.0 and self.gripper > self.gripper_min:
            # left -> close gripper
            self.gripper = max(self.gripper - 50, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing. PWM: {self.gripper}")



def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
