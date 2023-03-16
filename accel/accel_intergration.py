import time, sys, os, rospy
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
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

def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration

def get_accel():
    imu = IMUData()
    ax = imu.ax/9.8067
    ay = imu.ay/9.8067
    az = imu.az/9.8067
    return ax,ay,az

def get_gyro():
    imu = IMUData()
    wx = imu.wx
    wy = imu.wy
    wz = imu.wz
    return wx,wy,wz

def get_mag():
    imu = IMUData()
    mx = imu.mx
    my = imu.my
    mz = imu.mz
    return mx,my,mz
    
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

def imu_integrator():
    data_indx = 1 # index of variable to integrate
    dt_stop = 5 # seconds to record and integrate

    plt.style.use('ggplot')
    plt.ion()
    fig,axs = plt.subplots(3,1,figsize=(12,9))
    break_bool = False
    while True:
        # Reading and Printing IMU values 
        accel_array,t_array = [],[]
        print("Starting Data Acquisition")
        [axs[ii].clear() for ii in range(0,3)]
        t0 = time.time()
        loop_bool = False
        while True:
            try:
                ax,ay,az = get_accel()
                wx,wy,wz = get_gyro()
                mx,my,mz = get_mag()
                t_array.append(time.time()-t0)
                data_array = [ax,ay,az,wx,wy,wz,mx,my,mz]
                accel_array.append(accel_fit(data_array[data_indx],
                                             *accel_coeffs[data_indx]))
                if not loop_bool:
                    loop_bool = True
                    print("Start Moving IMU...")
            except:
                continue
            if time.time()-t0>dt_stop:
                print("Data Acquisition Stopped")
                break
            
        if break_bool:
            break

        # Signal Filtering
        Fs_approx = len(accel_array)/dt_stop
        b_filt,a_filt = signal.butter(4,5,'low',fs=Fs_approx)
        accel_array = signal.filtfilt(b_filt,a_filt,accel_array)
        accel_array = np.multiply(accel_array,9.80665)

        # Print Sample Rate and Accel
        # Integration Value
        print("Sample Rate: {0:2.0f}Hz".format(len(accel_array)/dt_stop))
        veloc_array = np.append(0.0,cumtrapz(accel_array,x=t_array))
        dist_approx = np.trapz(veloc_array,x=t_array)
        dist_array = np.append(0.0,cumtrapz(veloc_array,x=t_array))
        print("Displace in y-dir: {0:2.2f}m".format(dist_approx))
        axs[0].plot(t_array,accel_array,label="$"+mpu_labels[data_indx]+"$",
                    color=plt.cm.Set1(0),linewidth=2.5)
        axs[1].plot(t_array,veloc_array,
                    label="$v_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(1),linewidth=2.5)
        axs[2].plot(t_array,dist_array,
                    label="$d_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(2),linewidth=2.5)
        [axs[ii].legend() for ii in range(0,len(axs))]
        axs[0].set_ylabel('Acceleration [m$\cdot$s$^{-2}$]',fontsize=16)
        axs[1].set_ylabel('Velocity [m$\cdot$s$^{-1}$]',fontsize=16)
        axs[2].set_ylabel('Displacement [m]',fontsize=16)
        axs[2].set_xlabel('Time [s]',fontsize=18)
        axs[0].set_title("MPU9250 Accelerometer Integration",fontsize=18)
        plt.pause(0.01)
        plt.savefig("accel/accel_veloc_displace_integration.png",dpi=300,
                    bbox_inches='tight',facecolor="#FCFCFC")

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)

    # Accelerometer Gravity Calibration
    mpu_labels = ['a_x','a_y','a_z'] # gyro labels for plots
    cal_size = 1000 # number of points to use for calibration 
    old_vals_bool = True # True uses values from another calibration
    if not old_vals_bool:
        accel_coeffs = accel_cal() # grab accel coefficients
        np.save('accel/accel_coeffs.npy', accel_coeffs)
    else:
        accel_coeffs = np.load('accel/accel_coeffs.npy')
        print("Accel coefficients loaded from file")
        print(accel_coeffs)

    # Record new data 
    data = np.array([get_accel() for ii in range(0,cal_size)]) # new values

    # integration over time
    imu_integrator()