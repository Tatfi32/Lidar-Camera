from pathlib import Path
from distutils.dir_util import copy_tree
import shutil
from datetime import datetime


class LogWriter:
    def __init__(self, data_path):

        self.data_path = data_path
        self.results_path = Path(data_path) / 'results'
        if self.results_path.is_dir():
            shutil.rmtree(str(self.results_path), ignore_errors=True)
        self.results_path.mkdir()

        self.calib_path = self.results_path / 'calib'
        self.image_path = self.results_path / 'leftImage'
        self.lidar_path = self.results_path / 'velodyne_points'
        self.image_data_path = self.image_path / 'data'
        self.lidar_data_path = self.lidar_path / 'data'

        self.calib_path.mkdir()
        self.image_path.mkdir()
        self.lidar_path.mkdir()
        self.image_data_path.mkdir()
        self.lidar_data_path.mkdir()

        open(str(self.image_path / 'timestamps.txt'), "w+")
        open(str(self.lidar_path / 'timestamps.txt'), "w+")
        copy_tree(str(Path(self.data_path) / 'data' / 'frames' / 'calib'), str(self.calib_path))

        self.image_files = [x for x in (Path(self.data_path) / 'data' / 'frames').glob('*.bmp') if x.is_file()]
        self.indx = 0

    def save_images(self, yaml_img_name, image_time):
        for item in self.image_files:
            if item.stem == yaml_img_name:
                print('Saving camera data, file name =', yaml_img_name)
                shutil.copy(item, str(self.image_data_path))
        """
        with open(str(self.image_path / 'timestamps.txt'), "a+") as f:
            f.write("%s\n" % datetime.fromtimestamp(image_time).strftime('%Y-%m-%d %H:%M:%S.%f'))
        """

    def save_lidar_data(self, time_lidar, df):
        # print('Saving lidar data, file number =', self.indx)
        with open(str(self.lidar_path / 'timestamps.txt'), "a+") as f:
            f.write("%s\n" % time_lidar)
        path = (str(self.lidar_data_path / str(self.indx)) + '.csv')
        df.to_csv(path, header=False, index=False)
        self.indx += 1
