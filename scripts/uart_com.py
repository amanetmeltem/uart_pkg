#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from uart_pkg.msg import MotorPositions
from geometry_msgs.msg import Twist
import serial
import time

class CommunicationClass:
    def __init__(self):
        rospy.init_node('motor_positions_publisher', anonymous=True)

        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        self.motor_pub = rospy.Publisher('motor_positions', MotorPositions, queue_size=10)

        self.serialPort = serial.Serial(    port='/dev/ttyUSB1',    # Seri port adı
                                            baudrate=9600,          # Baud rate (STM32 ile aynı olmalı)
                                            bytesize=8,             # Veri biti
                                            parity='N',             # Parite biti yok
                                            stopbits=1,             # Stop bit
                                            timeout=1               # Okuma timeout süresi
                                        )
        
        self.serialPort.flushInput()
        self.serialPort.flushOutput()

        self.lineer_x = None
        self.angular_z = None

        self.rate = rospy.Rate(1) #8Hz

        self.motor_positions = MotorPositions()

    def cmd_vel_callback(self, msg):
        self.lineer_x = msg.linear.x
        self.angular_z = msg.angular.z

    def run(self):
        while not rospy.is_shutdown():

            ######################## RECEIVE THE MOTOR POSITION
            if self.serialPort.in_waiting >= 4:
                try:
                    received_data = self.serialPort.read(4)
                except serial.SerialException as e:
                    rospy.logerr("Serial read error: %s", e)
                    continue

                # left motor
                sign = -1.0 if received_data[0] else 1.0
                self.motor_positions.left_motor_position = sign * float(received_data[1])
                # right motor (fixed assignment)
                sign = -1.0 if received_data[2] else 1.0
                self.motor_positions.right_motor_position = sign * float(received_data[3])

                rospy.loginfo("Received data: left=%.2f, right=%.2f",
                              self.motor_positions.left_motor_position,
                              self.motor_positions.right_motor_position)
                self.motor_pub.publish(self.motor_positions)
            else:
                rospy.logwarn("Expected ≥4 bytes, got %d", self.serialPort.in_waiting)

            ######################## TRANSMIT THE MOTOR SPEED
            if self.lineer_x is not None and self.angular_z is not None:
                # determine status bits
                lx_status = 0 if self.lineer_x >= 0 else 1
                az_status = 0 if self.angular_z >= 0 else 1
                lx = abs(self.lineer_x)
                az = abs(self.angular_z)

                # clamp to 0–255
                lx_b = max(0, min(255, int(lx)))
                az_b = max(0, min(255, int(az)))

                data = bytes([lx_status, lx_b, az_status, az_b])
                try:
                    self.serialPort.write(data)
                    rospy.loginfo("Sent data: %s", list(data))
                except serial.SerialException as e:
                    rospy.logerr("Serial write error: %s", e)

            self.rate.sleep()

if __name__ == "__main__":
    try:
        uart_node = CommunicationClass()
        uart_node.run()
    except rospy.ROSInterruptException:
        pass
