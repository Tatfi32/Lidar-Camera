from __future__ import absolute_import
import os,re
import sys
import glob
import struct
import numpy as np
import os.path


"""
Takes raw PCAP folder and photo_folder with timestamp  from YAML (result of script 1) 
Add XYZD PCAP data to photo_folder according YAML data into "velodyne_points" subfolder 
"""

LASER_ANGLES = [-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16
DISTANCE_RESOLUTION = 0.002
ROTATION_MAX_UNITS = 36000

def print_help_and_exit1():
	print('Usage: .py [from pcap dir] [from yaml]')
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
def check_time(Video_Flow,yaml_dir):
    yaml_timestamps_path=os.path.join(yaml_dir,Video_Flow,'velodyne_points')
    yaml_start =[]
    yaml_end = []
    line_counter=0
    f = open(yaml_timestamps_path+'/timestamps_lidar.txt',"r")
    for line in f:
        line_counter +=1
        yaml_t = line.strip("[]\n")
        yaml_t = yaml_t.replace(".0","",2)
        yaml_t = yaml_t.split(',',1)
        yaml_start.append(int(yaml_t[0]))
        yaml_end.append(int(yaml_t[1]))
    return (line_counter, yaml_start, yaml_end)
    
def check_files_names(x):
    f_name = os.path.split(os.path.splitext(x)[0])[-1] 
    mat=re.match(r"(?P<flow>\S+)\.(?P<VideoFlow>\d+)\.(?P<VideoNumber>\d+)\.(?P<type>\d*)", f_name)
    for key,val in (mat.groupdict()).iteritems():
        if  key.startswith("VideoFlow"):
            VideoFlow=val
    return (VideoFlow)   

def dir_check (out):
    try:
        if os.path.exists(os.path.join(out, 'data')) is False:
            os.makedirs(os.path.join(out, 'data'))
        else:
            delete_files_in_dir(os.path.join(out, 'data'))
    except Exception, e:
         print e
                
def unpack_pcap(dirs,yaml_dir):
    files =glob.glob(dirs+'/*.pcap')
    points = []
    azimuth = None
    step_azimuth = None
    cur_Video_Flow = None
    
    for x in files:
        Video_Flow=check_files_names(x)
        if cur_Video_Flow!=Video_Flow:
            Video_Flow_path=os.path.join(yaml_dir, Video_Flow,'velodyne_points')
            dir_check(Video_Flow_path)
            line_counter,yaml_start,yaml_end=check_time(Video_Flow,yaml_dir)
            cur_Video_Flow=Video_Flow
            line = 0
        d = open(x, 'rb').read()  
        n = len(d)
        packet = d[24 : ]        #packet header and packet data  without global header
        for offset in xrange(0, n-24, 1264):
            if (n-offset) < 1264: break  
            data = packet[offset + 16 + 42 : offset + 16 + 42 + 1200 + 4 + 2]
            first_timestamp, factory = struct.unpack_from("<IH", data, offset=1200)  #timestamp for the first firing in the packet data
            assert hex(factory) == '0x2237', 'Error mode: 0x22=VLP-16, 0x37=Strongest Return'
            
            if (first_timestamp<yaml_start[line]):
                continue
            
            if (first_timestamp>yaml_end[line]) and (line<line_counter-1):
                line+=1
                continue
            
            if (first_timestamp<=yaml_end[line]) and  (first_timestamp>=yaml_start[line]):
                #print ('first_timestamp ', first_timestamp) 
                #print (yaml_start[line], yaml_end[line])
                #print ('line',line)
                seq_index = 0  
                for seq_offset in xrange(0, 1200, 100):
                    flag, first_azimuth = struct.unpack_from("<HH", data, seq_offset)
                    assert hex(flag) == '0xeeff' , 'Flag error'
                    for step in xrange(2):
                        if (step==0) and ((seq_index%2)==0) and (seq_index<22):
                            flag, third_azimuth = struct.unpack_from("<HH", data, seq_offset + 4 + 3 * 16 * 2  )
                            assert hex(flag) == '0xeeff' , 'Flag error'
                            if (third_azimuth<first_azimuth): 
                                step_azimuth=third_azimuth+ROTATION_MAX_UNITS-first_azimuth
                            else:
                                step_azimuth=third_azimuth-first_azimuth
                        arr = struct.unpack_from('<' + "HB" * NUM_LASERS, data, seq_offset + 4 + step * 3 * 16) 
                        for i in xrange(NUM_LASERS): 
                            azimuth = first_azimuth+ (step_azimuth * (55.296/1e6 * step +i * (2.304/1e6)))/(2 * 55.296/1e6)
                            if (azimuth>ROTATION_MAX_UNITS): 
                                azimuth-=ROTATION_MAX_UNITS        
                            if arr[i * 2] != 0:
                                points.append(calc_real_val(arr[i * 2], azimuth, i, arr[i * 2 + 1]))
                                #print (calc_real_val(arr[i * 2], azimuth, i, arr[i * 2 + 1]))
                        seq_index += 1 
                f = open(Video_Flow_path+'/data/'+str(line)+'.txt',"a+")
                for point in points:
                    f.write("%s\n" % point)
                f.close()  
                points = []
            else:
                break
      
def main():
    if len(sys.argv) == 3:
        print ('unpack PCAP folder:', sys.argv[1], ', with YAML from:', sys.argv[2])
        unpack_pcap(sys.argv[1],sys.argv[2])   
    else:
        print_help_and_exit1()

if __name__ == '__main__':
    main()