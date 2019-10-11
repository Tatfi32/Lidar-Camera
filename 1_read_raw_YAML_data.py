from __future__ import absolute_import
import os,re
#import calendar
import gzip
import sys,glob
from datetime import datetime
from distutils.dir_util import copy_tree
import yaml
import shutil


"""
Takes YAML data folder and directory with all photos
Create folder with photos and their timestamps from YAML data 
OUT folder structure
Video_name->
            calibration data from pictures folder
            Device Name_1 (data and time.txt)
            ....
            Device Name_n (data and time.txt)

"""


def print_help_and_exit():
	print('Usage: .py [from yaml bin file dir] [from good_images file dir] [to out directory]')
	sys.exit()

def delete_files_in_dir(path):
    files = glob.glob(path+'/*.txt')
    for f in files:
        os.remove(f)
        
def Frame_Counter(Dev_Name,Frame_Info,Global_Time):
    for info in Frame_Info:
        if info.startswith("grabMsec"):
            print (info)
            ts= Global_Time + int(info[len("grabMsec:"):])/1e3
            return(ts)
                    
def move_images(counter, framesdir,YAML_Image_name,dev_name, out, image_time):
    """
copy image files from framesdir folder to out folder with timestamp from YAML file
   structure of out folder:
        calib/ (copied calibration txts)
        VideoFlow/Device_name/data (copied image files)
        VideoFlow/Device_name/timestamp.txt
        """   
    start_path=os.path.join(out, dev_name)
    data_path=os.path.join(start_path, 'data')
    try:
        if (os.path.exists(start_path) is False):
            os.makedirs(start_path)
            if os.path.exists(os.path.join(start_path, 'data')) is False:
                os.makedirs(os.path.join(start_path, 'data'))
                data_path=os.path.join(start_path, 'data')
            if os.path.exists(os.path.join(start_path,  'timestamp.txt')) is True:
                os.remove(os.path.join(start_path, 'timestamp.txt'))
        elif (counter==0):
            delete_files_in_dir(start_path)
    except Exception, e:
        print e
        
    copy_tree(framesdir+"/calib", out+"/calib")
    for item in glob.iglob(os.path.join(framesdir, "*.bmp")):
        if ((os.path.split(os.path.splitext(item)[0])[-1]) == YAML_Image_name):
            shutil.copy(item, data_path)
    f=open(start_path+'/timestamps.txt',"a+")
    f.write("%s\n" % datetime.fromtimestamp(image_time).strftime('%Y-%m-%d %H:%M:%S.%f'))
    f.close() 
        
def get_image_names(framesdir):
    """
    FrameNumbers_good=[]
    VideoFlows_good=[]
    VideoNumbers_good=[]
    Devs_good=[]
    """
    """get the list of image names in images file folder"""
    Image_names = []
    for f in sorted(os.listdir(framesdir)):
        if os.path.splitext(f)[-1] not in {'.bmp', '.png', '.jpg'}:
            continue
        f_names = os.path.split(os.path.splitext(f)[0])[-1] 
        Image_names.append (f_names)
        
        """info from video name (not used)
        mat=re.match(r"(?P<flow>\S+)\.(?P<VideoFlow>\d+)\.(?P<VideoNumber>\d+)\.(?P<Dev>\S+)\.(?P<FrameNumber>\d*)", f_names)
        #print(mat.groupdict())
        for key,val in (mat.groupdict()).iteritems():
            if  key.startswith("FrameNumber"):
                FrameNumbers_good.append(val)
            if  key.startswith("VideoNumber"):
                VideoNumbers_good.append(val)
            if  key.startswith("VideoFlow"):
                VideoFlows_good.append(val)
            if key.startswith("Dev"):
                Devs_good.append(val)
        """
    return Image_names

""" lidar timestamps from yaml files"""
def save_lidar_timestamps(timestamp,timestamps_lidar, out, VideoFlow,counter):
    velodyne_path=os.path.join(out, VideoFlow, 'velodyne_points')
    try:
        if (os.path.exists(velodyne_path) is False):
            os.makedirs(velodyne_path)
        elif (counter==0):
            delete_files_in_dir(velodyne_path)
    except Exception, e:
        print e
        
    f1 = open(velodyne_path+'/timestamps_lidar.txt',"a+")
    f1.write("%s\n" % timestamps_lidar)
    f1.close()
    
    f = open(velodyne_path+'/timestamps.txt',"a+")
    f.write("%s\n" % timestamp)
    f.close()
    
def read_yaml(dirs,framesdir,out): 
    Image_names=get_image_names(framesdir)
    YAML_Image_names =[]
    VideoFlows=[]
    VideoNumbers=[]
    counter=0
    files = glob.glob(dirs+'\*.yml.gz')
    
    for x in files:
        f_names = os.path.split(os.path.splitext(x)[0])[-1] 
        mat=re.match(r"(?P<flow>\S+)\.(?P<VideoFlow>\d+)\.(?P<VideoNumber>\d+)\.(?P<info>\S+)\.(?P<type>\d*)", f_names)
        for key,val in (mat.groupdict()).iteritems():
            if  key.startswith("VideoNumber"):
                VideoNumbers.append(val)
            if  key.startswith("VideoFlow"):
                VideoFlows.append(val)   
        
        """read yaml file's data"""        
        with gzip.open(x,"r") as config:
            data = yaml.safe_load_all(config)
            c = config.read()
            leftImage_FrameNumber = 0
            if c.startswith("%YAML:1.0"):
                c = "%YAML 1.1" + c[len("%YAML:1.0"):] 
                data = list(yaml.load_all(c))
                DeviceNumbers = []
                DeviceNames = []
                header = data[0]['header']
                for key, value in header.iteritems():
                    """use for several Devices
                    if key.startswith("firstShotDate:"):
                        Header_Time=(key[len("firstShotDate::"):-7])
                        Header_Sec=calendar.timegm(datetime.strptime(Header_Time, "%Y-%m-%dT%H:%M:%S.%f").timetuple())
                    """
                    """use for leftImage Device"""
                    if key.startswith("captures"):
                        for captures in  value.iteritems():
                            for capture in captures:
                                if(type(capture) is str):
                                    DeviceNumbers.append(int(capture[len("captur:"):]))
                                        #print (DeviceNumbers)
                                elif type(capture) is dict:
                                    for val in capture:
                                        if val.startswith("name"):
                                            DeviceNames.append(val[len("name:"):])
                                                #print ('DeviceNames ', DeviceNames)
                """read shots yaml data"""
                Shot_counter = 0
                shots = data[0]['shots']
                for shot in shots:  
                    #grabNumber_shot = shot['grabNumber']
                    #grabMsec_shot=shot['grabMsec']
                    Shot_counter+=1
                    for key, value in shot.iteritems():
                        """use for several Devices
                        
                        if (key in DeviceNames):
                            Frame_Counter(key,value,Header_Sec)
                            #print(key, Frame_Counter(key,value,Header_Sec), Shot_counter)
                        """
                        """use for leftImage Device"""
                        if  key.startswith("leftImage"):
                           leftImage_FrameNumber +=1    

                    if len(shot)>3:
                        leftImages=shot['leftImage']
                        for key, value in leftImages.iteritems():
                            if  key.startswith("deviceSec"):
                                leftImage_deviceSec=int(key[len("deviceSec:"):])
                            if  key.startswith("grabMsec"):
                                leftImage_grabMsec=int(key[len("grabMsec:"):])
                        YAML_Image_names.append("new."+ str(VideoFlows[-1])+'.'+str(VideoNumbers[-1])+'.'+'left.'+str('%000006d'%leftImage_FrameNumber))
                        
                        if YAML_Image_names[-1] in Image_names:
                            Video_Flow =os.path.join(out, VideoFlows[-1])
                            move_images(counter, framesdir, YAML_Image_names[-1],DeviceNames[-1], Video_Flow,leftImage_grabMsec/1e6+leftImage_deviceSec)
                            """read lidar timestamps data """
                            velodyneLidars=shot['velodyneLidar']
                            for key, value in velodyneLidars.iteritems():
                                if key.startswith("grabMsec"):
                                    velodyneLidar_grabMsec=int(key[len("grabMsec:"):])
                                    #print ('velodyneLidar_grabMsec',velodyneLidar_grabMsec)
                                if key.startswith("lidarData"):
                                    pacTimeStamps = value["pacTimeStamps"]
                                    #print (pacTimeStamps)
                            time_photo_and_lidar = datetime.fromtimestamp(velodyneLidar_grabMsec/1e6+leftImage_deviceSec).strftime('%Y-%m-%d_%H_%M_%S.%f') 
                            timestamp_lidar = [pacTimeStamps[0],pacTimeStamps[-1]]
                            save_lidar_timestamps(time_photo_and_lidar, timestamp_lidar, out,VideoFlows[-1],counter)
                            counter+=1
                           
def main():
    if len(sys.argv) ==4:
        print ('from yaml folder', sys.argv[1], 'from images folder',sys.argv[2], 'to', sys.argv[3])
        read_yaml(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print_help_and_exit()
        sys.exit(2)
        
if __name__ == "__main__":
    main()