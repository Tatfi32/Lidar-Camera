from __future__ import absolute_import
import os,re
import sys
import glob
from datetime import datetime
import struct
import numpy as np
import os.path

LASER_ANGLES = [-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16
DISTANCE_RESOLUTION = 0.002
ROTATION_MAX_UNITS = 36000

def print_help_and_exit1():
	print('Usage: .py [from bin file dir] [to bin file dir] [camera direction azimuth]')
	sys.exit()

def calc_real_val(dis, azimuth, laser_id, inten):
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z,inten]

def delete_files_in_dir(path):
    files = glob.glob(path+'/*.txt')
    for f in files:
        os.remove(f)

def check_files_names(x):
    f_name = os.path.split(os.path.splitext(x)[0])[-1] 
    mat=re.match(r"(?P<flow>\S+)\.(?P<VideoFlow>\d+)\.(?P<VideoNumber>\d+)\.(?P<type>\d*)", f_name)
    for key,val in (mat.groupdict()).iteritems():
        if  key.startswith("VideoFlow"):
            VideoFlow=val
    return (VideoFlow)   


def dir_check (cur_Video_Flow,Video_Flow, out):
    if (cur_Video_Flow!=Video_Flow):
        try:
            if os.path.exists(os.path.join(out, 'data')) is False:
                    os.makedirs(os.path.join(out, 'data'))
            else:
                delete_files_in_dir(os.path.join(out, 'data'))
            if os.path.exists(os.path.join(out, 'timestamp')) is False:
                os.makedirs(os.path.join(out, 'timestamp'))
            else:
                delete_files_in_dir(os.path.join(out, 'timestamp'))
        except Exception, e:
            print e
                
def unpack_pcap(dirs,out,camera_azimuth):
    files =glob.glob(dirs+'/*.pcap')
    
    points = []
    azimuth = None
    max_azimuth = None
    step_azimuth = None
    azimuth_camera_time = 0
    prev_azimuth = None
    scan_index=-1
    az_step_frame_index = scan_index
    azimuth_step = ROTATION_MAX_UNITS
    cur_Video_Flow = None
    for x in files:
        Video_Flow=check_files_names(x)
        Video_Flow_path=os.path.join(out, Video_Flow)
        d = open(x, 'rb').read()  
        n = len(d)
        packet = d[24 : ]        #packet header and packet data  without global header
        dir_check(cur_Video_Flow,Video_Flow, Video_Flow_path)
        #read data packet №offset in pcap file        
        for offset in xrange(0, n-24, 1264):
            if (n-offset) < 1264: break  
            packet_header = packet[offset : offset + 16]
            ts_sec, ts_usec= struct.unpack_from("<II", packet_header, offset=0)
            #print ('pcap  packet ts_sec ', ts_sec)
            #print (' pcap packet ts_usec ',ts_usec/1e6)
            timestemp_pcap=float(ts_sec) + float(ts_usec/1e6)
            #print ('  ts_sec+ts_usec ',timestemp_pcap) 
            data = packet[offset + 16 + 42 : offset + 16 + 42 + 1200 + 4 + 2]
            #print (' data ',data.encode('hex'))
            first_timestamp, factory = struct.unpack_from("<IH", data, offset=1200)  #timestamp for the first firing in the packet data
            assert hex(factory) == '0x2237', 'Error mode: 0x22=VLP-16, 0x37=Strongest Return'
            #print ('   first_timestamp ', first_timestamp/1e6) 
            print ('   first_timestamp ', first_timestamp) 
            #print ('    ts_sec+ts_usec+first_timestamp ',timestemp_pcap+(first_timestamp/1e6)) 
            
            seq_index = 0  
            #read 12 seq_index (each 100 bytes)
            for seq_offset in xrange(0, 1200, 100):
                flag, first_azimuth = struct.unpack_from("<HH", data, seq_offset)
                assert hex(flag) == '0xeeff' , 'Flag error'
                #each seq_index contain 2 series with 16 laser points
                for step in xrange(2):
                    if (step==0) and ((seq_index%2)==0) and (seq_index<22):
                        flag, third_azimuth = struct.unpack_from("<HH", data, seq_offset + 4 + 3 * 16 * 2  )
                        assert hex(flag) == '0xeeff' , 'Flag error'
                        if (third_azimuth<first_azimuth): 
                            step_azimuth=third_azimuth+ROTATION_MAX_UNITS-first_azimuth
                        else:
                            step_azimuth=third_azimuth-first_azimuth
                    arr = struct.unpack_from('<' + "HB" * NUM_LASERS, data, seq_offset + 4 + step * 3 * 16) # read (dist and reflectivity) for each channel (from 16 lines) for 1/2 seq
                    #16 lidar points have different time and azimuth offset
                    for i in xrange(NUM_LASERS): 
                        time_offset = (55.296 * seq_index + 2.304 * i) /1e6
                        azimuth = first_azimuth+ (step_azimuth * (55.296/1e6 * step +i * (2.304/1e6)))/(2 * 55.296/1e6)
                        #print('    first_timestamp + time_offset', (timestemp_pcap+(first_timestamp + time_offset)/1e6, 'seq=',seq_index, 'step=', step ))
                        if (azimuth>ROTATION_MAX_UNITS): 
                                azimuth-=ROTATION_MAX_UNITS        
                        if arr[i * 2] != 0:
                            #points array with the real [X Y Z Reflection] values
                            points.append(calc_real_val(arr[i * 2], azimuth, i, arr[i * 2 + 1]))
                            print (calc_real_val(arr[i * 2], azimuth, i, arr[i * 2 + 1]))
                            # create file for start values of the laser time rotation (timestamps_start.txt)
                            if (prev_azimuth>35800) and (prev_azimuth>azimuth):
                                f= open(Video_Flow_path+'/timestamp/timestamps_start.txt',"a+")
                                f.write("%s\n" % datetime.fromtimestamp(timestemp_pcap+(time_offset)/1e6).strftime('%Y-%m-%d %H:%M:%S.%f'))
                                f.close() 
                                scan_index += 1 
                                print ('     scan_index=',scan_index)
                                
                            # create file for end values of the laser time rotation (timestamps_end.txt)      
                            if (third_azimuth<50):
                                if (azimuth>=max_azimuth):
                                    max_azimuth=azimuth
                                    max_azimuth_time=timestemp_pcap+(time_offset)/1e6
                                else:
                                    f= open(Video_Flow_path+'/timestamp/timestamps_end.txt',"a+")
                                    f.write("%s\n" % datetime.fromtimestamp(max_azimuth_time).strftime('%Y-%m-%d %H:%M:%S.%f'))
                                    f.close() 
                                    max_azimuth = None
                                
                            # create file for camera direction times values (timestamps.txt)
                            if ((abs(first_azimuth-camera_azimuth))<500) and (scan_index==(az_step_frame_index)) and (scan_index>=0):
                                azimuth_camera_time=timestemp_pcap+(time_offset)/1e6
                                #slower calculations
                                """
                                if ((abs(azimuth-camera_azimuth))<azimuth_step):
                                    azimuth_step=abs(azimuth-camera_azimuth)
                                    azimuth_camera_time=timestemp_pcap+(time_offset)/1e6 
                                 """  
                            elif (scan_index > az_step_frame_index) :
                                    f= open(Video_Flow_path+'/timestamp/timestamps.txt',"a+")
                                    if (azimuth_camera_time > 0):
                                        f.write("%s\n" % datetime.fromtimestamp(azimuth_camera_time).strftime('%Y-%m-%d %H:%M:%S.%f'))
                                    f.close() 
                                    #azimuth_step = ROTATION_MAX_UNITS
                                    az_step_frame_index=scan_index
                                    
                            #create file with the real [X Y Z Reflection] values (data_frame_number.txt)        
                            if (azimuth >= prev_azimuth):
                                f= open(Video_Flow_path+'/data/'+str(scan_index)+'.txt',"a+")
                                for point in points:
                                    f.write("%s\n" % point)
                                f.close()  
                                points = []
                
                        prev_azimuth = azimuth
                    seq_index += 1 
  
    #delete start rotation file (it's contain not full 0-360 azimuth)               
    cur_Video_Flow==Video_Flow
    scan_index=-1
    try:
        if os.path.exists(Video_Flow_path+'/data/-1.txt') is True:
            os.remove(Video_Flow_path+'/data/-1.txt')
    except Exception, e:
        print e
        
def main():
    if len(sys.argv) == 4:
        print ('unpack from', sys.argv[1], 'to', sys.argv[2], 'camera azimuth =', sys.argv[3])
        unpack_pcap(sys.argv[1],sys.argv[2],float(sys.argv[3]))   
    else:
        print_help_and_exit1()


if __name__ == '__main__':
    main()
