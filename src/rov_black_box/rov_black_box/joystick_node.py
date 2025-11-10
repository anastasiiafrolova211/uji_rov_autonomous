import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import MountControl
from mavros_msgs.msg import OverrideRCIn, StreamRate
from time import sleep
import numpy as np


class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        
        self.get_logger().info('Starting BlueROV joystick')
        # for thruster pwm commands
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        # subscribtion for joystick function
        self.subscription = self.create_subscription(Joy, 'joy', self.joyCallback, 10)
        # publish for camera tilting control
        self.mount_pub = self.create_publisher(MountControl, 'mount_control/command', 10)

        # Create timer for continuous control (20 Hz)
        self.timer = self.create_timer(0.05, self.continuous_control_callback)
        
        self.armDisarm(False)
        rate = 20  # 20 Hz -> was 25 Hz
        self.setStreamRate(rate)


        # Button state tracking for continuous control
        self.lb_held = False  # Left bumper for tilt up
        self.rb_held = False  # Right bumper for tilt down
        self.dpad_left_held = False  # D-pad left for dimming lights
        self.dpad_right_held = False  # D-pad right for brightening lights

        # initialize variables
        self.set_mode = [True, False, False]
        self.arming = False
        # for gropper control
        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # used pins
        # pins:
        # 1-8 are thrusters
        # 9 
        # 10
        # 11 light
        # 12
        # 13 gripper
        # 14
        # 15 
        # 16 camera tilting - as it is not a service bc we need to pub angle to topic -> /bluerov2/mount_control/command

        self.light_pin = 11.0
        self.gripper_pin = 13.0
        self.camera_servo_pin = 16.0

        #light values 
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.light_step = 15.0  # Continuous increment per timer tick
        
        # camera tilt 
        self.tilt = 0.0
        self.tilt_int = 0.0 # for keeping neutral horizontal position
        self.tilt_step = 2.0  # Continuous increment (degrees) per timer tick
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        # gripper
        self.gripper = 1150.0
        self.gripper_min = 1150.0
        self.gripper_max = 1580.0

        ## Initial test for the system
        self.run_initialization_test = False  # changed to False to avoid blinding everyone around
        if self.run_initialization_test:
            self.initialization_test()


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


    def continuous_control_callback(self):
        """
        Timer callback for continuous camera tilt and light control when buttons are held
        """
        changed_tilt = False
        changed_light = False
        
        # Continuous camera tilt control
        if self.lb_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.rb_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True
        
        # Continuous light control
        if self.dpad_right_held:
            self.light = min(self.light + self.light_step, self.light_max)
            changed_light = True
        elif self.dpad_left_held:
            self.light = max(self.light - self.light_step, self.light_min)
            changed_light = True
        
        # Publish updates
        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)
        if changed_light:
            self.send_servo_command(self.light_pin, self.light)


    def send_servo_command(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        
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

        client.call_async(request)
        self.get_logger().debug(f'Sent servo command pin {pin_number}, value {value}')


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
        
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")


    def send_camera_tilt_command(self, tilt_angle_deg):
        '''
        Publishes camera tilt angle to /bluerov2/mount_control/command (suppose)
        tilt_angle_deg (float) --> desired tilt in degrees
        '''
        msg = MountControl()
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 - value from documentation
        msg.pitch = tilt_angle_deg  # up/down tilt
        msg.roll = 0.0 
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0

        self.mount_pub.publish(msg)
        self.get_logger().debug(f"Camera tilt: {tilt_angle_deg:.1f}°")


    def joyCallback(self, data):
        ''' 
        Map the Joystick buttons according the bluerov configuration as descriped at
        [https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg]
        (https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg)
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

        btn_gripper_open = data.axes[2] # LT button (goes from -1.0 to 1.0 when pressed so threshold is set to +-0.5)
        btn_gripper_close = data.axes[5] # RT button (same here)
        
        btn_light = data.axes[6] # D-Pad left/right

        # Update button held states for continuous control
        self.lb_held = btn_camera_servo_up == 1
        self.rb_held = btn_camera_servo_down == 1
        self.dpad_left_held = btn_light == 1.0  # D-pad left
        self.dpad_right_held = btn_light == -1.0  # D-pad right

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

        # Camera reset to horizontal
        if btn_camera_rest:
            self.tilt = 0.0  # reset to horizontal
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt has been reseted")

        # control grippers open/close position
        rt_pressed = btn_gripper_close < -0.5 # 0.5 is just threshold so no need to push the button fully
        lt_pressed = btn_gripper_open > 0.5
        
        # RT -> open gripper (only trigger once when pressed, not continuously)
        if rt_pressed and not self.rt_was_pressed and self.gripper < self.gripper_max:
            self.gripper = min(self.gripper + 430, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing. PWM: {self.gripper}")
            
        # LT -> close gripper (only trigger once when pressed, not continuously)
        if lt_pressed and not self.lt_was_pressed and self.gripper > self.gripper_min:
            self.gripper = max(self.gripper - 430, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening. PWM: {self.gripper}")

        # Update trigger state for next callback
        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed


def timer_callback(self):
        '''
        Time step at a fixed rate (1 / timer_period = 20 Hz) to execute control logic.
        '''
        if self.set_mode[0]:  # commands sent inside joyCallback()
            self.correction_mode = False
            return
        elif self.set_mode[
            1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            self.correction_mode = False
            self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            return
        elif self.set_mode[2]:
            if  not self.correction_mode:
                self.Correction_yaw = 1500
                self.Correction_surge = 1500
                self.depth_flag = False
                self.correction_mode = True
            # send commands in correction mode
            self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, self.Correction_surge, 1500)
            # self.setOverrideRCIN(1500, 1500, self.Correction_depth, 1500, 1500, 1500) # only depth control for making videos

        else:  # nor
            pass


def velCallback(self, cmd_vel):
        ''' Used in manual mode to read the values of the analog and map it pwm then send it the thrusters'''
        if (self.set_mode[1] or self.set_mode[2]):
            return
        else:
            self.get_logger().info("Sending...")

        # Extract cmd_vel message
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
        
        # send the commands to the mthrusters 
        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)
        


def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set 
        '''
        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500 # camera pan servo motor speed 
        msg_override.channels[7] = 1500 #camers tilt servo motro speed

        self.pub_msg_override.publish(msg_override)

def setStreamRate(self, rate):
        ''' Set the Mavros rate for reading the senosor data'''
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info("stream rate requested, wait_for_service, (False if timeout) result :" + str(result))

        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("set stream rate Succeeded")


def mapValueScalSat(self, value):
        ''' Map the value of the joystick analog form -1 to 1 to a pwm value form 1100 to 1900
            where 1500 is the stop value 1100 is maximum negative and 1900 is maximum positive'''
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
