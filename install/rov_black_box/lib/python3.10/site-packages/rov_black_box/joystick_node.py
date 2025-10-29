import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandLong
from time import sleep

class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        
        self.get_logger().info('Starting BlueROV joystick controller...')
        self.subscription = self.create_subscription(Joy, '/bluerov2/joy', self.joyCallback, 10)

        # Initialize variables
        self.set_mode = [True, False, False]
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False

        self.light_pin = 13.0
        self.camera_servo_pin = 15.0
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.tilt = 1450.0
        self.servo_min = 1100.0
        self.servo_max = 1850.0

        self.run_initialization_test = True
        if self.run_initialization_test:
            self.initialization_test()

    def initialization_test(self):
        self.get_logger().info("Testing light and camera servo...")
        self.light = self.light_max
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_command(self.light_pin, self.light)
        sleep(0.5)
        self.send_servo_command(self.camera_servo_pin, self.servo_max)
        sleep(0.5)
        self.send_servo_command(self.camera_servo_pin, self.servo_min)
        self.get_logger().info("Initialization test complete.")

    def send_servo_command(self, pin_number, value):
        client = self.create_client(CommandLong, 'cmd/command')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return
        
        req = CommandLong.Request()
        req.command = 183
        req.param1 = pin_number
        req.param2 = value
        
        future = client.call_async(req)
        self.get_logger().info(f'Sent servo command pin {pin_number}, value {value}')

    def armDisarm(self, armed):
        cli = self.create_client(CommandLong, 'cmd/command')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MAVROS service not available!')
            return

        req = CommandLong.Request()
        req.command = 400
        req.param1 = 1.0 if armed else 0.0
        cli.call_async(req)
        self.get_logger().info('Vehicle armed' if armed else 'Vehicle disarmed')

    def joyCallback(self, data):
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_corrected_mode = data.buttons[0]
        btn_camera_servo_up = data.buttons[4]
        btn_camera_servo_down = data.buttons[5]
        btn_camera_rest = data.buttons[9]
        btn_light_down = data.axes[2]
        btn_light_up = data.axes[5]

        # Arming/Disarming
        if btn_disarm == 1 and self.arming:
            self.arming = False
            self.armDisarm(False)
        if btn_arm == 1 and not self.arming:
            self.arming = True
            self.armDisarm(True)

        # Mode switching
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.get_logger().info("Mode manual")
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.get_logger().info("Mode automatic")
        elif btn_corrected_mode and not self.set_mode[2]:
            self.set_mode = [False, False, True]
            self.get_logger().info("Mode correction")

        # Lights
        if btn_light_up == -1 and self.light < self.light_max:
            self.light = min(self.light + 100.0, self.light_max)
            self.send_servo_command(self.light_pin, self.light)
        elif btn_light_down == -1 and self.light > self.light_min:
            self.light = max(self.light - 100.0, self.light_min)
            self.send_servo_command(self.light_pin, self.light)

        # Camera tilt
        if btn_camera_servo_up and self.tilt < self.servo_max:
            self.tilt = min(self.tilt + 100, self.servo_max)
            self.send_servo_command(self.camera_servo_pin, self.tilt)
        elif btn_camera_servo_down and self.tilt > self.servo_min:
            self.tilt = max(self.tilt - 100, self.servo_min)
            self.send_servo_command(self.camera_servo_pin, self.tilt)
        elif btn_camera_rest:
            self.tilt = 1450.0
            self.send_servo_command(self.camera_servo_pin, self.tilt)
            self.get_logger().info("Camera tilt reset")

def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
