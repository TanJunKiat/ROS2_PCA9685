# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

print("Initializing IO System - import")
import time
import adafruit_pca9685
from adafruit_servokit import ServoKit
import serial
import board
import busio

global thrinit, strinit, maxr, minl, maxthr, minthr

kit = ServoKit(channels=16, address=0x40)
print("Initializing IO System - kit")

i2c = busio.I2C(board.SCL, board.SDA)
print("Initializing IO System - board")
pca = adafruit_pca9685.PCA9685(i2c)
print("Initializing IO System - pca")

#kit.set_pwm_freq(50)
#pca.setPWMFreq(50)
print("Initializing IO System - freq")
pca.frequency = 400


maxthr=15 # Max Throttle PWM
minthr= 0 # Min Throttle PWM


print("Initializing Propulsion System") #
kit.servo[0].angle = 0 # this Throttle Servo number will need to be set
kit.servo[1].angle = 0 # this Throttle Servo number will need to be set
kit.servo[2].angle = 0 # this Throttle Servo number will need to be set
kit.servo[3].angle = 0 # this Throttle Servo number will need to be set
kit.servo[4].angle = 0 # this Throttle Servo number will need to be set
kit.servo[5].angle = 0 # this Throttle Servo number will need to be set
time.sleep(1) # wait 1 second for the ESC to initialize zero

print("Listener Node Started...")

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        t1 = msg.linear.x
        t2 = msg.linear.y
        t3 = msg.linear.z
        t4 = msg.angular.x
        t5 = msg.angular.y
        t6 = msg.angular.z
                
        kit.servo[0].angle = t1/maxthr*180
        kit.servo[1].angle = t2/maxthr*180
        kit.servo[2].angle = t3/maxthr*180
        kit.servo[3].angle = t4/maxthr*180
        kit.servo[4].angle = t5/maxthr*180
        kit.servo[5].angle = t6/maxthr*180
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
