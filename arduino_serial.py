#!/bin/python3

import serial
import rospy
from std_msgs.msg import String

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def imu_publisher(Hz=100):
    pub = rospy.Publisher('imu_sensor', String, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(Hz)

    while not rospy.is_shutdown():
        line = ser.readline()
        data = line.decode('unicode_escape')
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException: pass

    ser.close()