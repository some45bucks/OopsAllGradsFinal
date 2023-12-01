
import subprocess
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node


LEFT_STICK_TURN_AXIS = 0
FORWARD_AXIS = 1
TURN_AXIS = 2
RIGHT_TRIGGGER = 5
LEFT_TRIGGGER = 4
LEFT_RIGHT_DPAD = 6
UP_DOWN_DPAD = 7

A_BUTTON = 0
B_BUTTON = 1
X_BUTTTON = 2
Y_BUTTON = 3
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
SELECT_BUTTON = 6
START_BUTTTON = 7

MAX_VEL = 2.33
MAX_ANGULAR_VEL = 5


class WirelessController(Node):
    def __init__(self):
        super().__init__("wireless_control")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(Joy, "/joy", self.handle_controller, 10)
        self.trigger_control = False
        self.current_max_vel = 1.2
        self.current_max_angular_vel = 1.2
        self.swapped_recently = False
        self.waiting_time_to_allow_switch = 30

        self.last_vel_sent = 0.0
        self.timer = self.create_timer(0.1, self.handle_pub)
        self.twist_msg = None

    
    def handle_pub(self):
        if self.twist_msg != None:

            # Make sure that the speed doesn't increase too fast at once
            if self.twist_msg.linear.x > 0.0 and self.last_vel_sent != self.twist_msg.linear.x:
                self.last_vel_sent += 0.2
                self.last_vel_sent = min(self.twist_msg.linear.x, self.last_vel_sent)
                self.twist_msg.linear.x = self.last_vel_sent

            elif self.twist_msg.linear.x < 0.0 and self.last_vel_sent != self.twist_msg.linear.x:
                self.last_vel_sent -= 0.2
                self.last_vel_sent = max(self.twist_msg.linear.x, self.last_vel_sent)
                self.twist_msg.linear.x = self.last_vel_sent
            
            elif self.twist_msg.linear.x == 0.0:
                self.last_vel_sent = 0.0
            
            self.publisher.publish(self.twist_msg)

    # Forward: msg.axes[1]
    # 
    def handle_controller(self, msg: Joy):
        if msg.buttons[SELECT_BUTTON] == 1 and msg.buttons[START_BUTTTON] == 1 and not self.swapped_recently:
            self.trigger_control = not self.trigger_control
            self.swapped_recently = True
            self.waiting_time_to_allow_switch = 30
        elif self.swapped_recently:
            self.waiting_time_to_allow_switch -= 1
            if self.waiting_time_to_allow_switch <= 0:
                self.swapped_recently = False
        
        # if msg.buttons[X_BUTTTON] == 1:
        #     if not self.is_recording:
        #         self.start_recording()
        #     else:
        #         self.stop_recording()

        if msg.buttons[Y_BUTTON] == 1:
            self.save_map()

        if msg.axes[UP_DOWN_DPAD] != 0.0:
            if msg.axes[UP_DOWN_DPAD] > 0:
                old_max_vel = self.current_max_vel
                self.current_max_vel = self.current_max_vel + self.current_max_vel * .10
                self.current_max_vel = max(self.current_max_vel, 0)

            else:
                old_max_vel = self.current_max_vel

                self.current_max_vel = self.current_max_vel - self.current_max_vel * .10
                self.current_max_vel = min(self.current_max_vel, MAX_VEL)

            print(f"Increasing from {old_max_vel} m/s to {self.current_max_vel}")


        if msg.axes[LEFT_RIGHT_DPAD] != 0.0:
            old_max_angular_vel = self.current_max_angular_vel

            if msg.axes[LEFT_RIGHT_DPAD] > 0:
                self.current_max_angular_vel = self.current_max_angular_vel - self.current_max_angular_vel * .10
                self.current_max_angular_vel = max(self.current_max_angular_vel, 0)
            else:
                self.current_max_angular_vel = self.current_max_angular_vel + self.current_max_angular_vel * .10
                self.current_max_angular_vel = min(self.current_max_angular_vel, MAX_ANGULAR_VEL)
            
            print(f"Increasing from {old_max_angular_vel} m/s to {self.current_max_angular_vel}")

        
        twist_message = Twist()
        

        # =======================================================================
        # Control Logic (Not trigger control)
        # =======================================================================
        if not self.trigger_control:
            twist_message.linear.x = msg.axes[FORWARD_AXIS] * self.current_max_vel
            twist_message.angular.z = msg.axes[TURN_AXIS] * self.current_max_angular_vel 


        # =======================================================================
        # Control Logic (Trigger control)
        # =======================================================================
        else:
            pass


        self.twist_msg = twist_message

    def start_recording(self):
        self.process = subprocess.Popen(['ros2', 'bag', 'record', '-a'])
        self.is_recording = True

    def stop_recording(self):
        if self.process:
            self.process.terminate()
            self.process = None
        self.is_recording = False

    def save_map(self):

        ld = generate_save_map_launch()
        ls = launch.LaunchService()
        ls.include_launch_description(ld)
        ls.run()
        print(f"saved map")

def generate_save_map_launch():

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', '/home/wheeltec/map'],
        
        parameters=[{'save_map_timeout': 20000},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)

    return ld

def main():
    rclpy.init()
    minimal_publisher = WirelessController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()