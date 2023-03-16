import time, sys, os, rospy
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

def outlier_removal(x_ii,y_ii):
    x_diff = np.append(0.0,np.diff(x_ii)) # looking for outliers
    stdev_amt = 5.0 # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff)>np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    x_inliers  = np.where(np.abs(x_diff)<np.abs(np.mean(x_diff))+\
                          (stdev_amt*np.std(x_diff)))
    y_diff     = np.append(0.0,np.diff(y_ii)) # looking for outliers
    y_outliers = np.where(np.abs(y_diff)>np.abs(np.mean(y_diff))+\
                          (stdev_amt*np.std(y_diff)))
    y_inliers  = np.abs(y_diff)<np.abs(np.mean(y_diff))+\
                 (stdev_amt*np.std(y_diff))
    if len(x_outliers)!=0:
        x_ii[x_outliers] = np.nan # null outlier
        y_ii[x_outliers] = np.nan # null outlier
    if len(y_outliers)!=0:
        y_ii[y_outliers] = np.nan # null outlier
        x_ii[y_outliers] = np.nan # null outlier
    return x_ii,y_ii

def get_mag():
    imu = IMUData()
    mx = imu.mx
    my = imu.my
    mz = imu.mz
    return mx,my,mz

def mag_cal():
    print("-"*50)
    print("Magnetometer Calibration")
    mag_cal_rotation_vec = [] # variable for calibration calculations
    for qq,ax_qq in enumerate(mag_cal_axes):
        input("-"*8+" Press Enter and Start Rotating the IMU Around the "+ax_qq+"-axis")
        print("\t When Finished, Press CTRL+C")
        mag_array = []
        t0 = time.time()
        while True:
            try:
                mx,my,mz = get_mag()
                rospy.spin()
            except KeyboardInterrupt:
                break
            except:
                continue
            mag_array.append([mx,my,mz]) # mag array
        mag_array = mag_array[20:] # throw away first few points (buffer clearing)
        mag_cal_rotation_vec.append(mag_array) # calibration array
        print("Sample Rate: {0:2.0f} Hz".format(len(mag_array)/(time.time()-t0)))
        
    mag_cal_rotation_vec = np.array(mag_cal_rotation_vec) # make numpy array
    ak_fit_coeffs = []
    indices_to_save = [0,0,1] # indices to save as offsets
    for mag_ii,mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags) # mag numpy array
        x,y = mags[:,cal_rot_indices[mag_ii][0]],\
                        mags[:,cal_rot_indices[mag_ii][1]] # sensors to analyze
        x,y = outlier_removal(x,y) # outlier removal
        y_0 = (np.nanmax(y)+np.nanmin(y))/2.0 # y-offset
        x_0 = (np.nanmax(x)+np.nanmin(x))/2.0 # x-offset
        ak_fit_coeffs.append([x_0,y_0][indices_to_save[mag_ii]]) # append to offset
        
    return ak_fit_coeffs,mag_cal_rotation_vec

# Plot Values to See Calibration Impact
def mag_cal_plot():
    plt.style.use('ggplot') # start figure
    fig,axs = plt.subplots(1,2,figsize=(12,7)) # start figure
    for mag_ii,mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags) # magnetometer numpy array
        x,y = mags[:,cal_rot_indices[mag_ii][0]],\
                    mags[:,cal_rot_indices[mag_ii][1]]
        x,y = outlier_removal(x,y) # outlier removal 
        axs[0].scatter(x,y,
                       label='Rotation Around ${0}$-axis (${1},{2}$)'.\
                    format(mag_cal_axes[mag_ii],
                           mag_labels[cal_rot_indices[mag_ii][0]],
                           mag_labels[cal_rot_indices[mag_ii][1]]))
        axs[1].scatter(x-mag_coeffs[cal_rot_indices[mag_ii][0]],
                    y-mag_coeffs[cal_rot_indices[mag_ii][1]],
                       label='Rotation Around ${0}$-axis (${1},{2}$)'.\
                    format(mag_cal_axes[mag_ii],
                           mag_labels[cal_rot_indices[mag_ii][0]],
                           mag_labels[cal_rot_indices[mag_ii][1]]))
    axs[0].set_title('Before Hard Iron Offset') # plot title
    axs[1].set_title('After Hard Iron Offset') # plot title
    mag_lims = [np.nanmin(np.nanmin(mag_cal_rotation_vec)),
                np.nanmax(np.nanmax(mag_cal_rotation_vec))] # array limits
    mag_lims = [-1.1*np.max(mag_lims),1.1*np.max(mag_lims)] # axes limits
    for jj in range(0,2):
        axs[jj].set_ylim(mag_lims) # set limits
        axs[jj].set_xlim(mag_lims) # set limits
        axs[jj].legend() # legend
        axs[jj].set_aspect('equal',adjustable='box') # square axes
    fig.savefig('mag/mag_cal_hard_offset_white.png',dpi=300,bbox_inches='tight',
                facecolor='#FFFFFF') # save figure
    plt.show() #show plot

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    
    # Magnetometer Calibration
    mag_labels = ['m_x','m_y','m_z'] # mag labels for plots
    mag_cal_axes = ['z','y','x'] # axis order being rotated
    cal_rot_indices = [[0,1],[1,2],[0,2]] # indices of heading for each axis
    mag_coeffs,mag_cal_rotation_vec = mag_cal() # grab mag coefficients

    # Plot with and without offsets
    mag_cal_plot() # plot un-calibrated and calibrated results
    
        