import cv2
import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image,Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL

def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0] #get the file of the
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

def ReadImages(filename):
    '''Generates a list of files from the directory'''
    print("Searching directory %s" % dir)
    all = []
    left_files = []
    right_files = []
    files = os.listdir(filename)
    timestamp=[]
    for f in sorted(files):
        if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg', '.pgm']:
            '''
            if 'left' in f or 'left' in path:
                left_files.append(os.path.join(path, f))
            elif 'right' in f or 'right' in path:
                right_files.append(os.path.join(path, f))
            '''
            all.append(os.path.join(filename, f))
            timestamp.append(f.split('.')[0])
    # print(timestamp)
    return all,timestamp

def ReadIMU(filename):
    '''return IMU data and timestamp of IMU'''
    file = open(filename,'r')
    all = file.readlines()
    timestamp = []
    imu_data = []
    for f in all:
        line = f.rstrip('\r\n').split(',')
        timestamp.append(line[0])
        imu_data.append(line[1:])
    return timestamp,imu_data


def CreateBag(args):#img,imu, bagname, timestamps
    '''read time stamps'''
    imgs ,imagetimestamps= ReadImages(args[0])
    # imagetimestamps=[]
    imutimesteps,imudata = ReadIMU(args[1]) #the url of  IMU data
    # file = open(args[3], 'r')
    # all = file.readlines()
    # for f in all:
    #     imagetimestamps.append(f.rstrip('\n').split(' ')[1])
    # file.close()
    '''Creates a bag file with camera images'''
    if not os.path.exists(args[2]):
        os.system(r'touch %s' % args[2])
        bag = rosbag.Bag(args[2], 'w')
    else:
        return

    try:
        for i in range(len(imudata)):
            imu = Imu()
            angular_v = Vector3()
            linear_a = Vector3()
            angular_v.x = float(imudata[i][0])
            angular_v.y = float(imudata[i][1])
            angular_v.z = float(imudata[i][2])
            linear_a.x = float(imudata[i][3])
            linear_a.y = float(imudata[i][4])
            linear_a.z = float(imudata[i][5])
            imuStamp = rospy.rostime.Time(long((imutimesteps[i].strip())[:-9]),long((imutimesteps[i].strip())[-9:]))
            imu.header.stamp=imuStamp
            imu.angular_velocity = angular_v
            imu.linear_acceleration = linear_a
            bag.write("IMU/imu_raw",imu,imuStamp)

        for i in range(len(imgs)):
            im=cv2.imread(imgs[i],1)
            cvimage=CvBridge().cv2_to_imgmsg(im)
            Stamp = rospy.rostime.Time(long((imagetimestamps[i].strip())[:-9]),long((imagetimestamps[i].strip())[-9:]))
            cvimage.header.stamp=Stamp
            cvimage.encoding="rgb8"
            # Img.data = Img_data
            bag.write('camera/image_raw',cvimage,Stamp)
    finally:
        bag.close()

if __name__ == "__main__":
    print(sys.argv)
    CreateBag(sys.argv[1:])
    #ReadImages(sys.argv[1])