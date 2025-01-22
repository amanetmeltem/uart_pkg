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
            if self.serialPort.in_waiting == 4:

                # received_data array has 4 elements, respectively: left_motor_position_status, left_motor_position, right_motor_position_status, right_motor_position
                # status = 0 means positive, status = 1 means negative
                # left_motor_position and right_motor_position are the absolute values of the left_motor_position and right_motor_position
                
                received_data = self.serialPort.read(4)

                if received_data[0] == 0:
                    self.motor_positions.left_motor_position = float(received_data[1])
                else:
                    self.motor_positions.left_motor_position = -float(received_data[1])
                if received_data[2] == 0:
                    self.motor_positions.left_motor_position = float(received_data[3])
                else:
                    self.motor_positions.left_motor_position = -float(received_data[3])

                print("Received data: left_motor_position = {}, right_motor_position = {}".format(self.motor_positions.left_motor_position, self.motor_positions.right_motor_position))
                print("-" * 20) 

                # Publishing motor position data
                self.motor_pub.publish(self.motor_positions)
            else:
                print("Received data format error!")

            ######################## TRANSMIT THE MOTOR SPEED

            if self.lineer_x is not None and self.angular_z is not None:
                
                # data array has 4 elements, respectively: lineer_x_status, lineer_x, angular_z_status, angular_z
                # status = 0 means positive, status = 1 means negative
                # lineer_x and angular_z are the absolute values of the lineer_x and angular_z

                lineer_x_status = 0
                angular_z_status = 0

                if self.lineer_x >= 0:
                    self.lineer_x_send = self.lineer_x
                    lineer_x_status = 0
                else:
                    self.lineer_x_send = -self.lineer_x
                    lineer_x_status = 1

                if self.angular_z >= 0:
                    self.angular_z_send = self.angular_z
                    angular_z_status = 0
                else:
                    self.angular_z_send = -self.angular_z
                    angular_z_status = 1

                # Ensure self.lineer_x and self.angular_z are within the range of 0 to 255
                # lineer_x_value = max(0, min(255, int(self.lineer_x)))
                # angular_z_value = max(0, min(255, int(self.angular_z)))

                data = bytes([lineer_x_status, int(self.lineer_x_send), angular_z_status, int(self.angular_z_send)])
                print()
                print("{} Sent data: lstatus = {}, lineer_x = {}, astatus = {}, angular_z = {}".format(rospy.Time.now(),data[0],data[1],data[2], data[3]))
                print("-*" * 20)
                self.serialPort.write(data)

            self.rate.sleep()

if __name__ == "__main__":
    try:
        uart_node = CommunicationClass()
        uart_node.run()
    except rospy.ROSInterruptException:
        pass
