#!/usr/bin/python
# Software License Agreement (BSD License)

import sys
import numpy
import argparse

import matplotlib
import matplotlib.pyplot as plt



numpy.set_printoptions(threshold=numpy.inf)



def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)


def plot_orb(stamps,traj, color_ ,label_):
    fig = plt.figure()
    x = []
    y = []
    for i in range(len(stamps)):
        x.append(traj[i,0])
        y.append(traj[i,2])
    plt.plot(x, y ,  '-' ,color=color_ , label=label_)
    plt.legend()
    return 



def plot_gps(stamps,traj, color_ ,label_):
    fig = plt.figure()
    x = []
    y = []
    for i in range(len(stamps)):
        x.append(traj[i,0])
        y.append(traj[i,1])
    plt.plot(x, y ,  '-' ,color=color_ , label=label_)
    plt.legend()
    return 



def plot_associated(first_stamps,first_xyz ):

    fig = plt.figure()
    x1 = []
    y1 = []
    for i in range(len(first_stamps)):
        x1.append(first_xyz[i,0]-first_xyz[0,0] )
        y1.append(first_xyz[i,1]- first_xyz[0,1])
    plt.plot(x1, y1 ,  '-' ,color='blue' , label="gps")
    plt.legend()
    plt.savefig("gps.png")

    fig = plt.figure()
    x2 = []
    y2 = []
    for i in range(len(first_stamps)):
        x2.append(first_xyz[i,4] - first_xyz[0,4])
        y2.append(first_xyz[i,5] - first_xyz[0,5])
    plt.plot(x2, y2,  '-' , color='red' , label="after aligned")
    plt.legend()
    plt.savefig("orb.png")

    fig = plt.figure()
    plt.plot(x1, y1 ,  '-' ,color='blue' , label="gps")
    plt.legend()
    plt.plot(x2, y2,  '-' , color='red' , label="after aligned")
    plt.legend()
    plt.savefig("asso.png")
    fig = plt.figure()
    add = fig.add_subplot(111)
    x3 = []
    y3 = []
    for i in range(len(first_stamps)):
        x3.append(first_xyz[i,4]-first_xyz[i,0] )
        y3.append(first_xyz[i,5] - first_xyz[i,1])
    add.plot(x3,'-' , color='blue' , label="x")
    add.plot(y3 ,'-' , color='red' , label="y")
    plt.legend()
    plt.savefig("x_y.png")

    return


if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('traj_type', help='choose traj type , orb ,gps ,associated')
    # parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    # parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    # parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    # parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    # parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    # parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    # parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    
    first_list = read_file_list(args.first_file)
    first_stamps = first_list.keys()
    first_stamps.sort()
    first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:7]] for a in first_stamps])

    first_stamps.sort()
    if (args.traj_type =='orb' ):
        print("orb")
        plot_orb(first_stamps,first_xyz, 'red', 'orb traj' )

    if (args.traj_type =='gps' ):
        plot_gps(first_stamps,first_xyz, 'blue', 'gps traj' )

    if (args.traj_type =='associated' ):
        plot_associated(first_stamps,first_xyz )

    plt.show()    

    # fig = plt.figure()
    # x1 = []
    # y1 = []
    # for i in range(len(first_stamps)):
    #     x1.append(first_xyz[i,0])
    #     y1.append(first_xyz[i,2])
    # plt.plot(x1, y1 ,  '-' ,color='blue' , label="gps")
    # plt.legend()
    # fig = plt.figure()
    # x2 = []
    # y2 = []
    # for i in range(len(first_stamps)):
    #     x2.append(first_xyz[i,4])
    #     y2.append(first_xyz[i,5])
    # plt.plot(x2, y2,  '-' , color='red' , label="after aligned")
    # plt.legend()

    # fig = plt.figure()
    # add = fig.add_subplot(111)
    # x3 = []
    # y3 = []
    # for i in range(len(first_stamps)):
    #     x3.append(first_xyz[i,4]-first_xyz[i,0] )
    #     y3.append(first_xyz[i,5] - first_xyz[i,1])
    # add.plot(x3,'-' , color='blue' , label="x")
    # add.plot(y3 ,'-' , color='red' , label="y")
    # plt.legend()
    # plt.show()






    #     if stamps[i]-last < 2*interval:
    #         x.append(traj[i][0])
    #         y.append(traj[i][1])
    #     elif len(x)>0:
    #         ax.plot(x,y,style,color=color,label=label)
    #         label=""
    #         x=[]
    #         y=[]
    #     last= stamps[i]
    # if len(x)>0:
    #     ax.plot(x,y,style,color=color,label=label)

    # print(first_xyz)
    # print(first_xyz[0,2])

    # for value in first_list[0:3]:
    # print(first_list )
    # for value in first_list[0:3]:
    #     print(value)
    # first_xyz = numpy.matrix([float(value) for value in first_list[0:3]] ).transpose()
    # second_list = associate.read_file_list(args.second_file)

    # matches = associate.associate(first_list, second_list,float(args.offset),float(args.max_difference))    
    # if len(matches)<2:
    #     sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")


    # first_xyz = numpy.matrix([[float(value) for value in first_list[a][0:3]] for a,b in matches]).transpose()
    # second_xyz = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for a,b in matches]).transpose()
    # rot,trans,trans_error = align(second_xyz,first_xyz)
    
    # second_xyz_aligned = rot * second_xyz + trans
    # print(rot)
    # print(trans)

    
    # first_stamps = first_list.keys()
    # first_stamps.sort()
    # first_xyz_full = numpy.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).transpose()
    
    # second_stamps = second_list.keys()
    # second_stamps.sort()
    # second_xyz_full = numpy.matrix([[float(value)*float(args.scale) for value in second_list[b][0:3]] for b in second_stamps]).transpose()
    # second_xyz_full_aligned = rot * second_xyz_full + trans
    
    # if args.verbose:
    #     print "compared_pose_pairs %d pairs"%(len(trans_error))

    #     print "absolute_translational_error.rmse %f m"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
    #     print "absolute_translational_error.mean %f m"%numpy.mean(trans_error)
    #     print "absolute_translational_error.median %f m"%numpy.median(trans_error)
    #     print "absolute_translational_error.std %f m"%numpy.std(trans_error)
    #     print "absolute_translational_error.min %f m"%numpy.min(trans_error)
    #     print "absolute_translational_error.max %f m"%numpy.max(trans_error)
    # else:
    #     print "%f"%numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error))
        
    # if args.save_associations:
    #     file = open(args.save_associations,"w")
    #     file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A)]))
    #     file.close()
        
    # if args.save:
    #     file = open(args.save,"w")
    #     file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(second_stamps,second_xyz_full_aligned.transpose().A)]))
    #     file.close()

    # if args.plot:
    #     import matplotlib
    #     matplotlib.use('Agg')
    #     import matplotlib.pyplot as plt
    #     import matplotlib.pylab as pylab
    #     from matplotlib.patches import Ellipse
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111)
    #     plot_traj(ax,first_stamps,first_xyz_full.transpose().A,'-',"black","ground truth")
    #     plot_traj(ax,second_stamps,second_xyz_full_aligned.transpose().A,'-',"blue","estimated")

    #     label="difference"
    #     for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
    #         ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
    #         label=""
            
    #     ax.legend()
            
    #     ax.set_xlabel('x [m]')
    #     ax.set_ylabel('y [m]')
    #     plt.show()
    #     plt.savefig(args.plot,dpi=90)
        
