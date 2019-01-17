import time
import sys
import os
import cv2
import numpy as np
import utm


def getGpsTime(filename):
    timestamps = list()
    index = list()
    file = open('/home/kotei/road/11.29/GPS.txt', 'r')
    gps=open('gps.log','w') 
    for line in file:
        lin = line.split()

        latitude = lin[1]
        longitude = lin[2]
        altitude=  lin[3]
        stat = lin[4]
        index.append(lin[5])
        timeTemp =2029 * 7* 86400.0 + float(lin[0]) + 315619200.000000
        timeTemp = str( "%.6f" %  timeTemp)
        timestamps.append(timeTemp)
        print (timeTemp)
        utm_xyz=utm.from_latlon(float(latitude),float(longitude))
        gps.write( timeTemp+' '+  str( "%.6f" %  utm_xyz[0])+ ' '+ str( "%.6f" %  utm_xyz[1])+' '+ altitude+ ' ' + stat +'\n')
        gps.flush()
    zipped = zip(index, timestamps)
    gps = np.array(list(zipped))
    return gps


def get(videoName, filename, timeList):
    dir = os.path.splitext(videoName)[0]
    if not os.path.exists(dir):
        os.makedirs(dir)
    videoCapture = cv2.VideoCapture(videoName)
    fps = videoCapture.get(cv2.CAP_PROP_FPS)
    success, frame = videoCapture.read()
    count = 0
    while success:
        # cv2.imshow("Oto Video", frame)
        success, frame = videoCapture.read()
        cv2.waitKey(100000000)

        if (count >= int(timeList[0, 0])):
            print(count)
            cameraindex = int(timeList[0, 0]) - count
            filename = dir + "/" + timeList[cameraindex, 1] + ".png"
            cv2.imwrite(filename, frame)

            count += 1
            print(count)
        else:
            # print(count)
            count += 1


if __name__ == '__main__':

    for path, names, files in os.walk(os.getcwd()):
        for f in files:
            if os.path.splitext(f)[1] in ['.txt']:
                fileGps =  os.path.join(path, f)

    timeList = getGpsTime(fileGps)

    video_files = list()
    for path, names, files in os.walk(os.getcwd()):
        for f in files:
            if os.path.splitext(f)[1] in ['.avi', 'mp4']:
                video_files.append(os.path.join(path, f))
    for file in video_files:
        get(file, fileGps, timeList)
