from pathlib import Path

from pcap_reader import *
from yaml_reader import *
from logger import *
from smi_calculation import *

if __name__ == '__main__':
    """
    
    Path().absolute() directory contain "data" folder 
    with "pcap", "yml" and "frames" folders inside (each folder with the relevant files type inside)
    
    1) local_yml_reader.power(range_yml) takes the number of "yml" files for processing in
    2) local_yml_reader.read_yml(self, filename) function, where "yml" devided into shots and
    3) shot_processing(self, shot) function parse each shot from shots with fuction:
    
        3.1) self.processing.get_all_points_from_pcap(pcap_index=self.VideoNumbers): 
            aggregate "pcap" data into dataframe (for one "pcap" file)
    
        3.2) self.camera_timestamps_processing(shot['leftImage'].items()) 
            collect shot info about time and frame in "yml shot"
        
        3.3) self.writer.save_images(yaml_img_name=yaml_img_name, 
        3.4) image_time=(leftImage_grabMsec / 1e6 + leftImage_deviceSec), 
        3.5) self.writer.save_lidar_data(time_lidar=time_lidar, df=csv_data_timestamp) 
            save image and correspond lidar data from one shot
            
    """

    #local_yml_reader = YML_Read(Path().absolute())
    # local_yml_reader.power()
    local_calibrator = SMI_calculations(Path().absolute()/ 'results')
    local_calibrator.power()
