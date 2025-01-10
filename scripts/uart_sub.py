#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from uart_pkg.msg import my_uart
import serial
import time


def func(uart):
    if (uart.data == ''):
       print("~~~")
    else:
        rospy.loginfo('alinan veriler data: (%s)', uart.data)
    #if (uart.y == ''):
       #print("~~~")
    #else:
        #rospy.loginfo('alinan veriler y: (%s)', uart.y)


rospy.init_node('subscriber',anonymous=True)
rospy.Subscriber('stm_topic',my_uart,func)
rospy.spin()