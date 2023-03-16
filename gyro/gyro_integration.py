import time, sys, os, rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
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
    gyro_labels = ['\omega_x','\omega_y','\omega_z'] # gyro labels for plots
    cal_size = 500 # points to use for calibration
    old_vals_bool = True # True uses values from another calibration
    if not old_vals_bool:
        gyro_offsets = gyro_cal() # calculate gyro offsets
        np.save('gyro/gyro_offsets.npy', gyro_offsets)
    else:
        gyro_offsets = np.load('gyro/gyro_offsets.npy')
        print("Gyro coefficients loaded from file")
        print(gyro_offsets)

    # Record new data 
    input("Press Enter and Rotate Gyro 360 degrees")
    print("Recording Data...")
    record_time = 5 # how long to record
    data,t_vec = [],[]
    t0 = time.time()
    while time.time()-t0<record_time:
        data.append(get_gyro())
        t_vec.append(time.time()-t0)
    samp_rate = np.shape(data)[0]/(t_vec[-1]-t_vec[0]) # sample rate
    print("Stopped Recording\nSample Rate: {0:2.0f} Hz".format(samp_rate))

    # Offset and Integration of gyro and plotting results
    rot_axis = 2 # axis being rotated (2 = z-axis)
    data_offseted = np.array(data)[:,rot_axis]-gyro_offsets[rot_axis]
    integ1_array = cumtrapz(data_offseted,x=t_vec) # integrate once in time
    
    # print out results
    print("Integration of {} in {}".format(gyro_labels[rot_axis],
                    gyro_labels[rot_axis].split("_")[1])+\
            "-dir: {0:2.2f}degrees".format(integ1_array[-1]))
    
    # plotting routine
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    axs[0].plot(t_vec,data_offseted,label="$"+gyro_labels[rot_axis]+"$")
    axs[1].plot(t_vec[1:],integ1_array,
                label=r"$\theta_"+gyro_labels[rot_axis].split("_")[1]+"$")
    [axs[ii].legend(fontsize=16) for ii in range(0,len(axs))]
    axs[0].set_ylabel('Angular Velocity, $\omega_{}$ [$^\circ/s$]'.format(gyro_labels[rot_axis].\
                                        split("_")[1]),fontsize=16)
    axs[1].set_ylabel(r'Rotation, $\theta_{}$ [$^\circ$]'.format(gyro_labels[rot_axis].\
                                            split("_")[1]),fontsize=16)
    axs[1].set_xlabel('Time [s]',fontsize=16)
    axs[0].set_title('Gyroscope Integration over 360$^\circ$ Rotation',
                        fontsize=18)
    fig.savefig('gyro/gyroscope_integration_360deg_rot.png',dpi=300,
                bbox_inches='tight',facecolor='#FFFFFF')
    plt.show()