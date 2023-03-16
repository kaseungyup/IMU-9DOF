import sys, os, rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
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

def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration

def get_accel():
    imu = IMUData()
    ax = imu.ax/9.8067
    ay = imu.ay/9.8067
    az = imu.az/9.8067
    return ax,ay,az
    
def accel_cal():
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = get_accel()
                    mpu_array.append([ax,ay,az]) # append to array
                    print("ax: ", ax, "ay: ", ay, "az: ", az)
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)

    # Accelerometer Gravity Calibration
    accel_labels = ['a_x','a_y','a_z'] # gyro labels for plots
    cal_size = 1000 # number of points to use for calibration
    
    use_prev_coeff = True
    if use_prev_coeff == False:
        accel_coeffs = accel_cal() # grab accel coefficients
        np.save('accel/accel_coeffs.npy', accel_coeffs)
    else:
        accel_coeffs = np.load('accel/accel_coeffs.npy')
    
    # Record new data 
    data = np.array([get_accel() for ii in range(0,500)]) # new values

    # Plot with and without offsets
    plt.style.use('ggplot')
    fig,axs = plt.subplots(2,1,figsize=(12,9))
    for ii in range(0,3):
        axs[0].plot(data[:,ii],
                    label='${}$, Uncalibrated'.format(accel_labels[ii]))
        axs[1].plot(accel_fit(data[:,ii],*accel_coeffs[ii]),
                    label='${}$, Calibrated'.format(accel_labels[ii]))
    axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
    axs[0].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
    axs[1].set_xlabel('Sample',fontsize=18)
    axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
    axs[0].set_title('Accelerometer Calibration Calibration Correction',fontsize=18)
    fig.savefig('accel/accel_calibration_output.png',dpi=300,
                bbox_inches='tight',facecolor='#FCFCFC')
    fig.show()