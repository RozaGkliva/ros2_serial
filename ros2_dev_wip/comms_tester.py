#!/usr/bin/python3
""" 
Node that interfaces the microcat mcu over serial port.
Handles bi-directional data transfer and publishing of relevant info.
used to test the communication between the microcat and the computer.

@author: Roza Gkliva
@contact: roza.gkliva@taltech.ee
@date: Jan. 2024


TODO: use the rm3 motor_module and hardware interface code as reference 
"""


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
        # incoming packet info
        self.packet_header = 'SYNC'
        self.packet_trailer = 'CNYS'
        self.header_length = len(self.packet_header)

        self.ser = serial.Serial(
            "/dev/ttyACM0",
            115200,
            timeout=0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

        self.ser.
        
        #set up timer to send to arduino
        self.transmitting_timer_period = 0.01  # for sending data to arduino at 100Hz
        self.create_timer(self.transmitting_timer_period, self.writeToArduino)
        

    def readFromArduino(self):
        pass

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
