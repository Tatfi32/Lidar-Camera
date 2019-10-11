from __future__ import print_function
from __future__ import absolute_import
import matplotlib.pyplot as plt
import os, sys, glob
from scipy.stats  import gaussian_kde
import numpy as np
import cv2

"""
Calculate SMI Metric with steepest gradient descent optimization
LiDAR feature - reflectivity
Image feature - grayscale intensity
"""

step = 0.01
def print_help_and_exit1():
    print('Usage: .py [from KITTY bin file dir]')
    sys.exit()

def read_img(data):
    img = cv2.imread(data)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    image = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return (image)

def show_image(images):
    for image in images:
        plt.figure()
        plt.imshow(image)
        #plt.show()

def Projection(x, y, z, alpha, beta, gamma, u0, v0, w0, K):
    """
(x, y, z) - scanned point in LiDAR coordinates
(u, v, w) - corresponding point  in camera coordinates
(u0, v0, w0, 1) - the translation vector (u0=0.885,v0=0, w0=1.535)  
R = Rroll*Rpitch*Ryaw -> 3*3 matrixes with the combination of cos and sin (alpha=0,betta=0,gamma=0)

 |u|    |R     t| |x|
 |v|  = |       | |y|
 |w|    |       | |z| 
        |0     1| |1|
(i, j) - return corresponding pint in the image plane (pinhole model):     
(i0, j0)read from K matrix from calibration file info
 |i|    |fx/w   0    i0| |u|
 |j|  = | 0    fy/w  j0| |v|
 |0|    |              | |1| 
    """
    roll = [[1, 0, 0], [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)]]
    pitch = [[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]]
    yaw = [[np.cos(gamma), -np.sin(gamma), 0], [np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]]
    R = np.dot(roll, np.dot(pitch, yaw)) 
    T = np.zeros([3, 4]) #translation vector
    T[:3, :3] = R[:, :]
    T[:, 3] = [u0, v0, w0]
    position = [x, y, z, 1]
    coord = np.dot(T, position) # (u, v, w)
    w = coord[2]
    coord[2] = 1
    P = np.zeros([3, 3])
    P[:, :] = K[:, :]
    P[0][0] = P[0][0] / w
    P[1][1] = P[1][1] / w
    pixels = np.dot(P, coord)
    if (abs(pixels[0]) < 961) and (abs(pixels[1]) < 541):
        return (pixels[:2])

def read_calib_data(calib_dir):
    cam_mono = cv2.FileStorage(calib_dir + "/cam_mono.yml", cv2.FILE_STORAGE_READ)
    K = cam_mono.getNode("K")
    return (K.mat())

def get_intensivity(pixel, img):
    x = int(pixel[0] + 959)
    y = int(pixel[1] + 539)
    return img[y][x]

def calc_val(line):
    line = line.strip("[]\n")
    line = line.split(',', 3)
    X = float(line[0])
    Y = float(line[1])
    Z = float(line[2])
    inten = int(line[3])
    return ([X, Y, Z, inten])

def read_pcap(lidar_file):
    pcap_data = []
    f = open(lidar_file, "r")
    line_counter = 0
    for line in f:
        pcap_data.append(calc_val(line))
        line_counter += 1
    return (pcap_data)

def mutual_information(r,intens):
    SMI=0.0
    kernel_r = gaussian_kde(r)
    ref = kernel_r.evaluate(range(0,255))
    print ('ref',ref)
    kernel_i = gaussian_kde(intens)
    inte = kernel_i.evaluate(range(0,255))
    print ('inte',inte)
    #reflectivity = np.histogram(r, bins=255, range=(0.0, 255.0), density=True)
    #intensivity = np.histogram(intens, bins=255, range=(0.0, 255.0), density=True)
    mutual = np.histogram2d(r,intens,bins=255,range=[[0,255],[0,255]],density=True)
    for i in range(0,255):
        for j in range(0,255):
            SMI+=0.5* ref[i]*inte[j]*((mutual[0][i][j]/ref[i]*inte[j])-1)**2
    return (SMI)

def calc_mutual_info(alpha,beta,gamma,u0,v0,w0,K,Lidar_data, images):
    # Creating random variables
    ref = []
    intens = []
    for i in range(37):
        for j in range(len(Lidar_data[i])):
            pixels = Projection(Lidar_data[i][j][0], Lidar_data[i][j][1], Lidar_data[i][j][2], alpha, beta, gamma, u0, v0, w0, K)
            if pixels is not None:
                ref.append(Lidar_data[i][j][3])
                intens.append(get_intensivity(pixels, images[i]))
    SMI = mutual_information(ref, intens)
    return (SMI)

def calculate_gradient(alpha,beta,gamma,u0,v0,w0,K,Lidar_data, images,step = step):
    gradient = np.zeros(6)
    gradient[0] = (calc_mutual_info(alpha+step,beta,gamma,u0,v0,w0,K,Lidar_data, images)- calc_mutual_info(alpha-step,beta,gamma,u0,v0,w0,K,Lidar_data, images)) /2/step
    gradient[1] = (calc_mutual_info(alpha,beta+step,gamma,u0,v0,w0,K,Lidar_data, images)- calc_mutual_info(alpha,beta-step,gamma,u0,v0,w0,K,Lidar_data, images)) /2/step
    gradient[2] = (calc_mutual_info(alpha,beta,gamma+step,u0,v0,w0,K,Lidar_data, images)- calc_mutual_info(alpha,beta,gamma-step,u0,v0,w0,K,Lidar_data, images)) /2/step
    gradient[3] = (calc_mutual_info(alpha,beta,gamma,u0+step,v0,w0,K,Lidar_data, images)- calc_mutual_info(alpha,beta,gamma,u0-step,v0,w0,K,Lidar_data, images)) /2/step
    gradient[4] = (calc_mutual_info(alpha,beta,gamma,u0,v0+step,w0,K,Lidar_data, images)- calc_mutual_info(alpha,beta,gamma,u0,v0-step,w0,K,Lidar_data, images)) /2/step
    gradient[5] = (calc_mutual_info(alpha,beta,gamma,u0,v0,w0+step,K,Lidar_data, images)- calc_mutual_info(alpha,beta,gamma,u0,v0,w0-step,K,Lidar_data, images)) /2/step
    return (gradient)

def read_data(folder):
    Video_Flows = []
    for f in sorted(os.listdir(folder)):
        f_name = os.path.split(os.path.splitext(f)[0])[-1]
        Video_Flows.append(f_name)
    for Video_Flow in Video_Flows:
        Lidar_path = os.path.join(folder, Video_Flow, 'velodyne_points', 'data')
        pcap_files = glob.glob(Lidar_path + '/*.txt')
        Lidar_data = [read_pcap(i) for i in pcap_files]
        Camera_path = os.path.join(folder, Video_Flow, 'leftImage', 'data')
        images_files = glob.glob(Camera_path + '/*.bmp')
        images = [read_img(i) for i in images_files]
        K = np.array(read_calib_data(os.path.join(folder, Video_Flow, 'calib')))

        ''' 
        # SMI gradient part (falls in local maximums)
        cur = [0.05,0,0,-0.74,0.1,1.4]
        delta = 0.000001
        cur_MI = calc_mutual_info(cur[0],cur[1],cur[2],cur[3],cur[4],cur[5],K,Lidar_data, images)
        gradient = calculate_gradient(cur[0],cur[1],cur[2],cur[3],cur[4],cur[5],K,Lidar_data, images)
        print("Gradient:",gradient)
        while True:
            print("Current position:",cur,cur_MI)
            next = cur + gradient*step*10
            next_MI = calc_mutual_info(next[0],next[1],next[2],next[3],next[4],next[5],K,Lidar_data, images)
            print("Next position:",next,next_MI)
            if (next_MI > cur_MI):
                if (next_MI < cur_MI + delta):
                    break
                cur_MI = next_MI
                cur = next
            else:
                print("Calculating gradient")
                gradient = calculate_gradient(cur[0],cur[1],cur[2],cur[3],cur[4],cur[5],K,Lidar_data, images)
                print("New gradient",gradient)
        '''
        #behavior near ground truth  point
        #alpha=0,betta=0,gamma=0,u0=0.885,v0=0, w0=1.535
        points = np.zeros((2,21)) #range for changing start parameters
        for i in range(0,21,1):
            points[0][i] = float(i-10) / 200  #number of points in changing interval

        for i in range(0,21,1):
            print(i)
            points[1][i] = calc_mutual_info(0,0,0,-0.885,0,1.535-points[0][i],K,Lidar_data, images)

        for i in range(0,21,1):
            print(points[0][i], points[1][i])
        """
        calculate mutual info for all files in folder with ground truth parametrs
        for i in range(len(pcap_files)):
            ref = []
            intens = []
            for j in range(len(Lidar_data[i])):
                pixels = Projection(Lidar_data[i][j][0], Lidar_data[i][j][1], Lidar_data[i][j][2],
                                    0, 0, 0, 0.885, 0, -1.535, K)     #ex and int parameters
                if pixels is not None:
                    ref.append(Lidar_data[i][j][3])
                    intens.append(get_intensivity(pixels, images[i]))
            SMI=mutual_information(ref,intens)
            print ('SMI',SMI,'image file', i)
        """
def main():
    if len(sys.argv) == 2:
        print ('unpack from', sys.argv[1])
        read_data(sys.argv[1])
    else:
        print_help_and_exit1()

if __name__ == '__main__':
    main()