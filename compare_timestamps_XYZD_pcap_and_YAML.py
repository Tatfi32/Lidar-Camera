import os,sys
import glob
from datetime import datetime
import calendar

def print_help_and_exit1():
	print(' Usage: .py [from pcap folder] [to yaml folder]')
	sys.exit()
"""
Script just print times from pcap and yaml folder (manual check the time)
"""
def compare_times (filesdir, out):
    pcap_times=[]
    pcap_counter=0
    Video_Flows_pcap = glob.glob(filesdir)
    for Video_Flow_pcap in Video_Flows_pcap:
        pcap_time_path=os.path.join(filesdir,Video_Flow_pcap, 'timestamp')
        f = open(pcap_time_path+'/timestamps.txt',"r")
        for line in f:
            pcap_counter +=1
            pcap_t=calendar.timegm(datetime.strptime(line.strip(), "%Y-%m-%d %H:%M:%S.%f").timetuple())
            pcap_times.append(pcap_t)
        print (pcap_times)
    
    
    Video_Flows_yaml = glob.glob(filesdir)
    print (Video_Flows_yaml)
    for Video_Flow in Video_Flows_yaml:
        yaml_times=[]
        yaml_counter=0
        yaml_time_path=os.path.join(out, Video_Flow, 'velodyne_points')
        f = open(yaml_time_path+'/timestamps.txt',"r")
        for line in f:
            yaml_counter +=1
            yaml_t=calendar.timegm(datetime.strptime(line.strip(), "%Y-%m-%d %H:%M:%S.%f").timetuple())
            yaml_times.append(yaml_t)
        print (yaml_times)
        f.close()
    

def main():
    
    if len(sys.argv) == 3:
        print ('from pcap folder', sys.argv[1], 'to yaml folder', sys.argv[2])
        compare_times(sys.argv[1],sys.argv[2])   
    else:
        print_help_and_exit1()

if __name__ == '__main__':
    main()

    