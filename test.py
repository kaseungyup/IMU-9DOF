import sys, os, rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import serial
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from std_msgs.msg import String
from classes.timer import Timer

class IMUData():
    def __init__(self):
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.mx = 0.0
        self.my = 0.0
        self.mz = 0.0
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_imu = "imu_sensor"
        self.isReady_imu = False
        self.sub_imu = rospy.Subscriber(self.topic_sub_imu, String, self.callback)
        while self.isReady_imu is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_imu = True
        array = data.data.split()
        self.ax = float(array[0])
        self.ay = float(array[1])
        self.az = float(array[2])
        self.wx = float(array[3])
        self.wy = float(array[4])
        self.wz = float(array[5])
        self.mx = float(array[6])
        self.my = float(array[7])
        self.mz = float(array[8])


def get_accel():
    imu = IMUData()
    ax = imu.ax
    ay = imu.ay
    az = imu.az
    return ax,ay,az
    
if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)

    ax, ay, az = get_accel()
    print(ax, ay, az)
