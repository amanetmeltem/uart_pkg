#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from uart_pkg.msg import MotorPositions
import serial
import time

pub = rospy.Publisher('motor_positions',MotorPositions,queue_size=10)

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=9600,
                           bytesize=8, timeout=0)

rospy.init_node('motor_positions_publisher',anonymous= True)

rate = rospy.Rate(10)
a = 0
i = 0

while(not rospy.is_shutdown()):
    data = MotorPositions()
    uart_bilgi = serialPort.readline()
    # print(uart_bilgi)
    uart_bilgi1 = uart_bilgi.decode('utf-8')
    time.sleep(0.1)
    if (uart_bilgi1 != None):
        if (uart_bilgi1 != ''):
            print(uart_bilgi1)
    else:
        print("ERROR")
       

    rate.sleep()

# import rospy
# from uart_pkg.msg import MotorPositions
# from geometry_msgs.msg import Twist
# import serial
# import time

# class CommunicationClass:
#     def __init__(self):
#         rospy.init_node('motor_positions_publisher', anonymous=True)

#         rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

#         self.motor_pub = rospy.Publisher('motor_positions', MotorPositions, queue_size=10)

#         self.serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, bytesize=8, timeout=0)

#         self.lineer_x = None
#         self.angular_z = None

#         self.rate = rospy.Rate(8) #8Hz

#         self.motor_positions = MotorPositions()

#     def cmd_vel_callback(self, msg):
#         self.lineer_x = msg.linear.x
#         self.angular_z = msg.angular.z

#     def run(self):
#         while not rospy.is_shutdown():

#             ######################## RECEIVE THE MOTOR POSITION
#             uart_receive_encode_data = self.serialPort.readline()
#             uart_receive_decode_data = uart_receive_encode_data.decode('utf-8')
#             print(uart_receive_decode_data)
            
#             if uart_receive_decode_data == '' or uart_receive_decode_data is None:
#                 print("Packet is empty or something wrong!")
#                 continue
            
#             # Assuming the received data is formatted as "L,R"
#             motor_positions = uart_receive_decode_data.split(',')
#             if len(motor_positions) == 2:
#                 self.motor_positions.left_motor_position = float(motor_positions[0])
#                 self.motor_positions.right_motor_position = float(motor_positions[1])

#                 # Publishing motor position data
#                 self.motor_pub.publish(self.motor_positions)
#             else:
#                 print("Received data format error!")

#             ######################## TRANSMIT THE MOTOR SPEED
#             uart_transmit_decode_data = "{},{}".format(self.lineer_x, self.angular_z)
#             uart_transmit_encode_data = uart_transmit_decode_data.encode('utf-8')
#             self.serialPort.write(uart_transmit_encode_data)

#             self.rate.sleep()

# if __name__ == "__main__":
#     try:
#         uart_node = CommunicationClass()
#         uart_node.run()
#     except rospy.ROSInterruptException:
#         pass
