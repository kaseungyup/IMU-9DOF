import os
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, String

from classes.timer import Timer
from classes.visualizerclass import VisualizerClass

Hz = 50

class IMUData():
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accx = 0.0
        self.accy = 0.0
        self.accz = 0.0
        self.init_subscriber()

    def init_subscriber(self):
        self.topic_sub_imu = "imu_sensor"
        self.isReady_imu = False
        self.sub_imu = rospy.Subscriber(self.topic_sub_imu, String, self.callback)
        while self.isReady_imu is False: rospy.sleep(1e-3)

    def callback(self, data):
        self.isReady_imu = True
        array = data.data.split()
        self.roll = float(array[2])
        self.pitch = -float(array[1])
        self.yaw = float(array[0])
        self.accx = float(array[3])
        self.accy = float(array[4])
        self.accz = float(array[5])

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    print("Start visualization_engine.")

    tmr_plot = Timer(_name='Plot',_HZ=Hz,_MAX_SEC=np.inf,_VERBOSE=True)
    imu = IMUData()
    V = VisualizerClass(name='simple viz',HZ=Hz)

    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (HZ)

            V.reset_markers()
            
            print("roll: %.2f, pitch: %.2f, yaw: %.2f"%(imu.roll, imu.pitch, imu.yaw))
            V.append_marker(x=0,y=0,z=0,frame_id='map',roll=imu.roll,pitch=imu.pitch,yaw=imu.yaw,
                scale=Vector3(0.2,0.06,0.06),color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)
            
            V.publish_markers()

        rospy.sleep(1e-8)

    V.delete_markers()