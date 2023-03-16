import sys, os, rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String

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
        try:
            self.ax = float(array[0])
            self.ay = float(array[1])
            self.az = float(array[2])
            self.wx = float(array[3])
            self.wy = float(array[4])
            self.wz = float(array[5])
            self.mx = float(array[6])
            self.my = float(array[7])
            self.mz = float(array[8])
        except:
            pass

def get_gyro():
    imu = IMUData()
    wx = imu.wx
    wy = imu.wy
    wz = imu.wz
    return wx,wy,wz

def gyro_cal():
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    imu_array = []
    gyro_offsets = [0.0,0.0,0.0]
    while True:
        try:
            wx,wy,wz = get_gyro() # get gyro vals
            print("wx: ", wx," wy: ", wy," wz: ", wz)
        except:
            continue

        imu_array.append([wx,wy,wz])

        if np.shape(imu_array)[0]==cal_size:
            for qq in range(0,3):
                gyro_offsets[qq] = np.mean(np.array(imu_array)[:,qq]) # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    
    # Gyroscope Offset Calculation
    gyro_labels = ['w_x','w_y','w_z'] # gyro labels for plots
    cal_size = 500 # points to use for calibration

    use_prev_coeff = False
    if use_prev_coeff == False:
        gyro_offsets = gyro_cal() # calculate gyro offsets
        np.save('gyro/gyro_offsets.npy', gyro_offsets)
    else:
        gyro_offsets = np.load('gyro/gyro_offsets.npy')

    # Record new data 
    data = np.array([get_gyro() for ii in range(0,cal_size)]) # new values

    # Plot with and without offsets
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    for ii in range(0,3):
        axs[0].plot(data[:,ii],
                    label='${}$, Uncalibrated'.format(gyro_labels[ii]))
        axs[1].plot(data[:,ii]-gyro_offsets[ii],
                    label='${}$, Calibrated'.format(gyro_labels[ii]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    axs[1].set_ylabel('$w_{x,y,z}$ [$^{\circ}/s$]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
    axs[0].set_title('Gyroscope Calibration Offset Correction',fontsize=22)
    fig.savefig('gyro/gyro_calibration_output.png',dpi=300,
                bbox_inches='tight',facecolor='#FCFCFC')
    fig.show()