
#!/usr/bin/env python
import os
import sys
import time
import csv
import numpy as np
import argparse
import shutil


# structure
# dataset/cam0/TIMESTAMP.png
# dataset/camN/TIMESTAMP.png
# dataset/imu.csv


def getGpsTime(filename):
    timestamps = list()
    index = list()
    file = open(filename, 'r')
    gps = open('gps.log','w')
    orb = open('orb.log','w')
    num = 0
    for line in file:
        lin = line.split()
        timeTemp= lin[0]
        #timestamps.append(timeTemp)
        #index.append(num)
        if (10 < num < 10000)&(num%2 ==0):
            orb.write( lin[0]+ "\n")
            gps.write( line)
            orb.flush()
            gps.flush()
        num=num+1

    #zipped = zip(index, timestamps)
    #gps = np.array(list(zipped))
    return 


def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    # image_files.append(os.path.join(path, f))
                    image_files.append(os.path.join(path, f))
                    timeTmp = os.path.splitext(f)[0]
                    # timemin = int(timeTmp[0:10]) 
                    timesec= int(timeTmp[-17:])
                    # timeNow = float( "%.6f" %( timemin +  timesec))
                    timestamps.append(timesec)
    # sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files


def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    print ( dir)
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:
                print ( folder)
                if folder[0:3] == "cam":
                    cam_folders.append(folder)
    return cam_folders


def renameImg(camfolder,image_files,fileGps ):
    count = 0

    dir = os.getcwd() + '/'+ camfolder
    if not os.path.exists(dir):
        os.makedirs(dir)

    for image_filename in image_files:
        # filename = os.path.split(image_filename)[0] +'/'+ fileGps[count, 1] + ".jpg"
        filename = os.getcwd() + '/'+ camfolder + '/'+  str(fileGps[count, 1]) + ".jpg"
        print(  filename   )
        print(image_filename)
        shutil.copyfile(image_filename,filename)
        # os.rename(image_filename,filename )
        count =count+1
    return






if __name__ == '__main__':
    # for path, names, files in os.walk(os.getcwd()):
    #     for f in files:
    #         if os.path.splitext(f)[1] in ['.txt']:
    #             fileGps =  os.path.join(path, f)
    fileGps = "./gps.txt"
    timeList = getGpsTime(fileGps)
    
 
            

