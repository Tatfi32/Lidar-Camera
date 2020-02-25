import struct
from pathlib import Path
import numpy as np
import time
import progressbar


class Timer:
    def __enter__(self):
        self.start = time.clock()
        return self

    def __exit__(self, *args):
        self.end = time.clock()
        self.interval = self.end - self.start

class PointReader:

    def __init__(self, data_path):
        self.data_path = Path(data_path) / 'data'
        self.pcap_path = self.data_path / 'pcap'
        self.pcap_files = np.sort([x for x in self.pcap_path.glob('*.*') if x.is_file()])
        self.LASER_ANGLES = [-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
        self.NUM_LASERS = 16
        self.DISTANCE_RESOLUTION = 0.002
        self.ROTATION_MAX_UNITS = 36000

        self.pcap_file_list = []
        self.seq_list = []

        self.azimuth_bin = 100
        self.first_timestamp = None
        self.factory = None

    def read_pcap(self, file_number):
        with Timer() as t:
            self.get_pcap_data(file_number)
        print('PointReader.read_pcap finished in %.03f s' % t.interval)
        return self.pcap_file_list

    def get_pcap_data(self, file_number):

        """
        Args:
            file_number: number of pcap file to be processed

        Function description:
            goes throw the all binary symbol inside pcap file
            call seq_processing for each sequence in packets structure:

            pcap file structure:
                0 - 24   <->  global header <-> not read
                24 - end <-> packets with 1264-binary symbol info

            packets structure:
                0 - 16      <->  local header <-> not read
                16 - 58     <->  42 binary symbol of meta info <-> not read
                58  - 1258  <->  12 sequences with 100 binary symbol each with dist and azimuth
                1258 - 1264 <->  4 binary symbol for timestamp and 2 factory bites

        """
        pcap_file = str(self.pcap_files[int(file_number) - 1])
        pcap_data = open(pcap_file, 'rb').read()
        pcap_data = pcap_data[24:]

        with progressbar.ProgressBar(max_value=int(len(pcap_data))) as bar:
            for offset in range(0, int(len(pcap_data)), 1264):
                bar.update(offset)

                if (len(pcap_data) - offset) < 1264:
                    break

                cur_packet = pcap_data[offset + 16: offset + 16 + 42 + 1200 + 4 + 2]
                cur_data = cur_packet[42:]

                self.first_timestamp, self.factory = struct.unpack_from("<IH", cur_data, offset=1200)
                assert hex(self.factory) == '0x2237', 'Error mode: 0x22=VLP-16, 0x37=Strongest Return'
                seq_index = 0

                for seq_offset in range(0, 1100, 100):
                    self.seq_processing(cur_data, seq_offset, seq_index, self.first_timestamp, int(file_number))

                if self.seq_list:
                    self.pcap_file_list += self.seq_list
                    self.seq_list = []

    def seq_processing(self, data, seq_offset, seq_index, first_timestamp, file_number):
        """

        Args:
            data: current sequence
            seq_offset: number of the sequence in packet (0 - 1200 )
            seq_index: number of current sub-sequence in sequence (0- 22)
            first_timestamp: 4 binary symbol of timestamp
            file_number: number of pcap file to be processed

        Function description:
        unpack sequence data into seq_row_list for each laser_id block
        with {'X', 'Y', 'Z', 'D', 'azimuth', 'laser_id','first_timestamp', 'pcap_num'} info

        seq_row_list pushed into self.seq_list


            each sequence contain 2*11 sub-sequence
                sub-sequence structure:
                    flag
                    first_azimuth
                    16 laser_id block of binary symbol

        """
        flag, first_azimuth = struct.unpack_from("<HH", data, seq_offset)
        step_azimuth = 0

        assert hex(flag) == '0xeeff', 'Flag error'

        seq_row_list = []
        for step in range(2):
            if step == 0 and seq_index % 2 == 0 and seq_index < 22:
                flag, third_azimuth = struct.unpack_from("<HH", data, seq_offset + 4 + 3 * 16 * 2)

                assert hex(flag) == '0xeeff', 'Flag error'

                if third_azimuth < first_azimuth:
                    step_azimuth = third_azimuth + self.ROTATION_MAX_UNITS - first_azimuth
                else:
                    step_azimuth = third_azimuth - first_azimuth
            arr = struct.unpack_from('<' + "HB" * self.NUM_LASERS, data, seq_offset + 4 + step * 3 * 16)

            for i in range(self.NUM_LASERS):
                azimuth = first_azimuth + (step_azimuth * (55.296 / 1e6 * step + i * 2.304 / 1e6)) / (2 * 55.296 / 1e6)
                if azimuth > self.ROTATION_MAX_UNITS:
                    azimuth -= self.ROTATION_MAX_UNITS
                x, y, z = self.calc_real_val(arr[i * 2], azimuth, i)
                # azimuth_time = (55.296 / 1e6 * step + i * (2.304 / 1e6)) + first_timestamp
                if y > 0:
                    d = arr[i * 2 + 1]
                    azimuth_v = round(azimuth * 1.0 / self.azimuth_bin)
                    seq_row_list.append({'X': x, 'Y': y, 'Z': z, 'D': d, 'azimuth': azimuth_v, 'laser_id': i,
                                         'first_timestamp': first_timestamp, 'file_number': file_number})
            seq_index += 1
        self.seq_list += seq_row_list

    def calc_real_val(self, dis, azimuth, laser_id):
        r = dis * self.DISTANCE_RESOLUTION
        omega = self.LASER_ANGLES[laser_id] * np.pi / 180.0
        alpha = (azimuth / 100.0) * (np.pi / 180.0)
        x = r * np.cos(omega) * np.sin(alpha)
        y = r * np.cos(omega) * np.cos(alpha)
        z = r * np.sin(omega)
        return x, y, z


class PointProcessing:
    def __init__(self, data_path):
        self.data_path = data_path
        self.local_point_reader = PointReader(data_path=self.data_path)
        self.processed_pcap = None
        self.pcap_file_list = []

    def get_all_points_from_pcap(self, pcap_index):
        """

        Args:
            pcap_index:
                number of pcap file in folder (==yml VideoNumbers)

        Function description:

            when get new pcap_index:
                    unpack pcap data into dataframe with self.local_point_reader.read_pcap(pcap_index)

            if pcap file have already processed pcap_index:
                return:
                    dataframe with unpacked lidar information from 1 pcap file

            Dependent functions:

                self.local_point_reader.read_pcap(pcap_index)
                return:
                        list with unpacked lidar data for one pcap file

                """

        if pcap_index != self.processed_pcap:
            self.processed_pcap = pcap_index
            self.pcap_file_list = self.local_point_reader.read_pcap(pcap_index)
            return self.pcap_file_list

        if pcap_index == self.processed_pcap:
            return self.pcap_file_list
