import re
import gzip
from datetime import datetime
import yaml
from pathlib import Path
import pandas as pd
import numpy as np

from pcap_reader import PointProcessing
from logger import LogWriter

pd.options.mode.chained_assignment = None

class YML_Read:

    def __init__(self, data_path):
        """

        Args:
            data_path:
            path to the Data_folder
            Data_folder include "pcap", "yml" and "frames" folders, each with the relevant files type inside
        """

        self.path = Path(data_path) / 'data'
        self.yml_path = self.path / 'yml'
        self.images = self.path / 'frames'

        self.yml_files = np.sort([x for x in self.yml_path.glob('*.gz*') if x.is_file()])
        self.image_files = [x.stem for x in self.images.glob('*.bmp') if x.is_file()]

        self.processing = PointProcessing(data_path)
        self.writer = LogWriter(data_path)

        self.processed_images = []
        self.VideoFlows = None
        self.VideoNumbers = None
        self.time_lidar = None
        self.image_number = 0
        self.pcap_file_df = pd.DataFrame()

    def power(self, range_yml=None):
        """

        Args:
            range_yml:
            number of yml file to be processed
            if None: all files in folder

        Function description:
            goes through all yml in range_yml
            create for each file dataframe with {'X','Y', 'Z', 'D',
                        'azimuth', 'laser_id', 'first_timestamp', 'file_number'} colums

        Dependent functions:
            self.regular_expression(yml_file.stem) - read yml file name
            self.read_yml(yml_file path) - read each yml file data
        """
        print('Running main loop...')
        if range_yml is None:
            range_yml = len(self.yml_files)
        for i in np.arange(range_yml):
            self.pcap_file_df = pd.DataFrame({'X': [], 'Y': [], 'Z': [], 'D': [],
                                              'azimuth': [], 'laser_id': [], 'first_timestamp': [], 'file_number': []})
            yml_file = self.yml_files[i]
            file_name = yml_file.stem

            print(f'Reading {file_name} ...')
            self.regular_expression(yml_file=file_name)
            self.read_yml(filename=yml_file)

    def regular_expression(self, yml_file):
        """

        Args:
            yml_file:
            name of the yml_file

        Function description:
            get current VideoNumbers and VideoFlows for the current yml file according yml file name

            example:
            new.003.001.info.yml.gz <-> yml file name
            VideoNumbers <-> 003
            VideoFlows <-> 001

        """
        mat = re.match(r"(?P<flow>\S+)\.(?P<VideoFlow>\d+)\.(?P<VideoNumber>\d+)\.(?P<info>\S+)\.(?P<type>\d*)",
                       yml_file)
        mat = mat.groupdict()
        self.VideoNumbers = mat["VideoNumber"]
        self.VideoFlows = mat["VideoFlow"]

    def read_yml(self, filename):
        """

        Args:
            filename:
                path to the yml file

        Function description:
            devide full yml file into shots
            each shot processed with self.shot_processing(shot=shot)

        Dependent functions:
            self.shot_processing(shot=shot)
        """
        with gzip.open(filename, "rt") as file:
            config = file.read()
            self.image_number = 0
            if config.startswith("%YAML:1.0"):
                config = "%YAML 1.1" + str(config[len("%YAML:1.0"):])
                data = list(yaml.safe_load_all(config))

                for shot in (data[0]['shots']):
                    self.shot_processing(shot=shot)

    def shot_processing(self, shot):
        """

        Args:
            shot: dictionaries  with  lidar-vlp16 and camera info in one shot

        Function description:
            read key and value in shot dictionary

            if key include lidar info:
                sub-dictionary with lidar data processed with (processing.get_all_points_from_pcap)

            if key include camera info:
                each camera sub-dictionary processed with (self.camera_timestamps_processing)

                if image name in "frames" folder and program already processed lidar sub-dictionary:

                    saving image with self.writer.save_images in "results" folder
                    extracting pcap data from the dataframe and saving them via self.writer.save_lidar_data

        Dependent functions:

        processing.get_all_points_from_pcap
        return:
                dataframe for current pcap data via checking yml VideoNumbers

        self.camera_timestamps_processing
        return:
                yaml_img_name - standard form of the image name according key number
                leftImage_deviceSec - image receiving time (Sec)
                leftImage_grabMsec - image receiving time (Msec)
        """
        pacTimeStamps = None
        for key, value in sorted(shot.items(), reverse=True):
            if key.startswith("velodyneLidar"):
                pacTimeStamps = shot['velodyneLidar']["lidarData"]["pacTimeStamps"]
                pcap_file_list = self.processing.get_all_points_from_pcap(pcap_index=self.VideoNumbers)
                if pcap_file_list:
                    self.pcap_file_df = pd.DataFrame(pcap_file_list)

            if key.startswith("leftImage"):
                yaml_img_name, leftImage_deviceSec, leftImage_grabMsec \
                    = self.camera_timestamps_processing(shot['leftImage'].items())

                if yaml_img_name in self.image_files and yaml_img_name not in self.processed_images:
                    self.processed_images.append(yaml_img_name)
                    if pacTimeStamps is not None:
                        self.writer.save_images(yaml_img_name=yaml_img_name, image_time=(leftImage_grabMsec / 1e6
                                                                                         + leftImage_deviceSec))
                        time_lidar = datetime.fromtimestamp(leftImage_grabMsec / 1e6
                                                            + leftImage_deviceSec).strftime('%Y-%m-%d_%H_%M_%S.%f')

                        csv_data_timestamp = self.pcap_file_df[
                            self.pcap_file_df['first_timestamp'] < int(pacTimeStamps[-1])]
                        csv_data_timestamp = csv_data_timestamp.drop_duplicates(["azimuth", "laser_id"], keep="last")
                        self.writer.save_lidar_data(time_lidar=time_lidar, df=csv_data_timestamp)

    def camera_timestamps_processing(self, camera_items):
        """

        Args:
            camera_items:
            key from shot with camera dictionary

        Returns:
            yaml_img_name - standard form of the image name according key number
            leftImage_deviceSec - image receiving time (Sec)
            leftImage_grabMsec - image receiving time (Msec))

        """
        leftImage_deviceSec = 0
        leftImage_grabMsec = 0

        yaml_img_name = ("new." + str(self.VideoFlows) + '.' + str(self.VideoNumbers) + '.' + 'left.'
                         + str('%000006d' % self.image_number))
        self.image_number += 1

        for key_Image, value_Image in camera_items:
            if key_Image.startswith("deviceSec"):
                leftImage_deviceSec = int(key_Image[len("deviceSec:"):])
            if key_Image.startswith("grabMsec"):
                leftImage_grabMsec = int(key_Image[len("grabMsec:"):])

        return yaml_img_name, leftImage_deviceSec, leftImage_grabMsec
