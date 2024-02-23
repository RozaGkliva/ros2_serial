#!/usr/bin/python3

""" 
Node that interfaces a serial device, reads an incoming packet and publishes the data in ROS.

dev notes:
- testing with ROS 2 Foxy

@author: Roza Gkliva
@contact: roza.gkliva@taltech.ee
@date: Jan. 2024

TODO: use the rm3 motor_module and hardware interface code as reference 
TODO: make code more modular: params come from yaml file, can talk to any serial device (within reason)
"""

# Copyright 2024 Roza Gkliva
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



import struct

import rclpy
import serial
import transforms3d
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu


class CommsTester(Node):

    def __init__(self):
        super().__init__("node_comms_tester")

        # set up serial connection to arduino
        self.declare_parameter("serial_params.port", "/dev/ttyUSB0")
        self.declare_parameter("serial_params.baud", 9600)
        self.serial_port = self.get_parameter("serial_params.port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("serial_params.baud").get_parameter_value().integer_value

        self.get_logger().info(f'baud: {self.baudrate}, port: {self.serial_port}')

        # incoming packet info
        self.packet_header = 'SYNCSYNC'
        self.packet_trailer = '\n'
        self.header_length = len(self.packet_header)

        self.ser = serial.Serial(
            self.serial_port,
            self.baudrate,
            timeout=0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )
        
        #set up timer to send to arduino
        self.transmitting_timer_period = 0.01  # for sending data to arduino at 100Hz
        self.create_timer(self.transmitting_timer_period, self.writeToArduino)

        self.reading_timer_period = 0.1  # for reading data from arduino at 100Hz
        self.create_timer(self.reading_timer_period, self.readFromArduino)
        

    def readFromArduino(self):
        # read from the serial port
        incoming_data = self.ser.read(1000)
        self.get_logger().info(f'incoming data: {incoming_data}')

    def writeToArduino(self):
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # convert quaternion to euler angles
        orientation = transforms3d.euler.quat2euler(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        )

        # sent the euler angles to the arduino
        outbuffer = 'sync'.encode('utf-8') 


def main(args=None):
    rclpy.init(args=args)
    node_comms_tester = CommsTester()
    rclpy.spin(node_comms_tester)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_comms_tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
