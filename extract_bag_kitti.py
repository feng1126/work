# -*- coding: utf-8 -*-
#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import os
import os.path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from novatel_msgs.msg import BESTPOS, CORRIMUDATA, INSCOV, INSPVAX

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys


class ImageCreator():
    def __init__(self):
        self.bridge = CvBridge()
        # create bagfile
        #if os.path.isdir('./image_0'):
        #    pass
        #    else:
        #        os.mkdir('./image_0')
        #if os.path.isdir('./image_1'):
        #pass
        #else:
        #os.mkdir('./image_1')
        fname='times.txt'
        fobj=open(fname,'w')   

        if not os.path.isdir('./image_0'):
                os.mkdir('./image_0')
        if not os.path.isdir('./image_1'):
                os.mkdir('./image_1')   
        with rosbag.Bag('/home/feng/2018-08-29-14-04-26_0.bag', 'r') as bag:  #要读取的bag文件；
            index1 = 0
            index2 = 0
            for topic,msg,t in bag.read_messages():
                if topic == "/left/image_raw_color": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        #image_name = "./image_0/L_" + timestr+ ".jpg" #图像命名：时间戳.jpg
                        #timestr = "%.6d" % index1
                        #index1 = index1 + 1
                        image_name = "./image_0/" + timestr+ ".png"
                        cv2.imwrite(image_name, cv_image)  #保存；
                        fobj.write(timestr+' \n')
                if topic == "/right/image_raw_color": #图像的topic；
                        try:
                            #cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"mono8")
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        #image_name = "./image_1/R_" + timestr+ ".jpg" #图像命名：时间戳.jpg
                        timestr = "%.6d" % index2
                        index2 = index2 + 1
                        image_name = "./image_1/" + timestr+ ".png"
                        cv2.imwrite(image_name, cv_image)  #保存；
                if topic == "/novatel_data/inspvax": #图像的topic；
                        timestr ="%.6f" %  t.to_sec()
                        latitude = "%.10f" % msg.latitude
                        longitude = "%.10f" % msg.longitude
                        fobj.write(timestr +' '+latitude+ ' '+longitude+'\n')

               
                        
if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
