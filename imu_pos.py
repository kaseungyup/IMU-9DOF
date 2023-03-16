import os
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, String
from collections import deque

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

    del_t = 1/Hz
    vel = np.array([[0], [0], [0]])
    pos = np.array([[0], [0], [0]])
    traj = deque()
    traj.append(pos)

    tmr_plot.start()
    while tmr_plot.is_notfinished(): # loop 
        if tmr_plot.do_run(): # plot (HZ)
            cx = np.cos(imu.roll); sx = np.sin(imu.roll)
            cy = np.cos(imu.pitch); sy = np.sin(imu.pitch)
            cz = np.cos(imu.yaw); sz = np.sin(imu.yaw)

            rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

            acc = np.array([[imu.accx], [imu.accy], [imu.accz]])
            acc_world = np.matmul(rx,np.matmul(ry,np.matmul(rz, acc))) - np.array(np.array([[0], [0], [9.8067]]))
            print("acc_world: ", np.transpose(acc_world))

            vel = vel + acc_world * del_t
            if tmr_plot.tick % 1500 == 0: pos = np.array([[0], [0], [0]])
            pos = pos + vel * del_t
            # print("pos: ", np.transpose(pos))
            traj.append(pos)
            if len(traj) > 50:
                traj.popleft()
            traj_arr = np.array(traj)

            V.reset_markers()
            V.reset_lines()
            
            # print("roll: %.2f, pitch: %.2f, yaw: %.2f"%(imu.roll, imu.pitch, imu.yaw))
            # print(pos)
            V.append_marker(x=pos[0],y=pos[1],z=pos[2],frame_id='map',roll=imu.roll,pitch=imu.pitch,yaw=imu.yaw,
                scale=Vector3(0.2,0.06,0.06),color=ColorRGBA(1.0,0.0,0.0,0.5),marker_type=Marker.ARROW)
            V.append_line(x_array=traj_arr[:,0],y_array=traj_arr[:,1],z_array=traj_arr[:,2],r=0.01,
                frame_id='map',color=ColorRGBA(0.0,0.0,1.0,1.0),marker_type=Marker.LINE_STRIP)
            V.publish_markers()
            V.publish_lines()

        rospy.sleep(1e-8)

    V.delete_markers()
    V.delete_lines()