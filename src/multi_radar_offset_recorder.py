#!/usr/bin/env python3

""" MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. """

import rospy
import json
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class RadarDataProcessor:
    def __init__(self, window_time_in_ms, grid_size, map_width, map_height, save_folder):
        self.window_time_in_ms = window_time_in_ms
        self.grid_size = grid_size
        self.map_width = map_width
        self.map_height = map_height
        self.grid_data = {}
        self.save_folder = save_folder

        self.received_data = {}
        self.last_window_time_ms = -1

        self.centroid = (0, 0)

    def callback_cloud(self, msg, radar_id):
        # self.process_radar_data(msg, radar_id)
        if radar_id not in self.received_data:
            self.received_data[radar_id] = []

        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        self.received_data[radar_id].extend(points)

    def loop(self):
        current_time_ms = float(rospy.get_rostime().to_nsec())/1000000.0
        if (current_time_ms>0):
            if (self.last_window_time_ms<0):
                self.last_window_time_ms = current_time_ms
            
            elapsed_in_ms = current_time_ms - self.last_window_time_ms


            if elapsed_in_ms>=self.window_time_in_ms:
                # print(f'elapsed_in_ms {elapsed_in_ms}')
                current_data = self.received_data.copy()
                global_centroid = self.calculate_centroid(current_data)
                print(f'Global centroid: {global_centroid}')
                self.calculate_grid_values(current_data, global_centroid)
                self.reset_data()
                self.last_window_time_ms = current_time_ms

    def calculate_centroid(self, current_data):
        total_points = []
        for radar_data in current_data.values():
            total_points.extend(radar_data)

        if total_points:
            x_sum = sum(point[0] for point in total_points)
            y_sum = sum(point[1] for point in total_points)
            total_points_count = len(total_points)
            return (x_sum / total_points_count, y_sum / total_points_count)
        else:
            return (0,0)

    def calculate_grid_values(self, current_data, global_centroid):
        for radar_id, radar_data in current_data.items():
            grid_data = self.grid_data.setdefault(radar_id, {})
            total_points = []
            total_points.extend(radar_data)
            if total_points:
                x_sum = sum(point[0] for point in total_points)
                y_sum = sum(point[1] for point in total_points)
                total_points_count = len(total_points)
                centroid = (x_sum / total_points_count, y_sum / total_points_count)
                diff_centroids = (centroid[0] - global_centroid[0], centroid[1] - global_centroid[1])
                grid_x = int(centroid[0] / self.grid_size)
                grid_y = int(centroid[1] / self.grid_size)
                grid_key = (grid_x, grid_y)
                print(f'radar: {radar_id} centroid: {centroid} diff: {diff_centroids}')
                grid_data.setdefault(grid_key, []).append((diff_centroids[0], diff_centroids[1]))

    def reset_data(self):
        self.received_data = {}

    def mean_corrections_by_grid(self):
        for radar_id, grid_data in self.grid_data.items():
            for grid_key, corrections in grid_data.items():
                total_points = []
                total_points.extend(corrections)
                x_sum = sum(point[0] for point in total_points)
                y_sum = sum(point[1] for point in total_points)
                total_points_count = len(total_points)
                mean = (x_sum / total_points_count, y_sum / total_points_count)
                grid_data[grid_key] = mean



    def interpolate_missing_values(self):
        for radar_id, grid_data in self.grid_data.items():
            for grid_key, points in grid_data.items():
                if not points:
                    self.interpolate_grid_value(radar_id, grid_key)

    def interpolate_grid_value(self, radar_id, grid_key):
        neighbors = [(grid_key[0] + dx, grid_key[1] + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]]
        valid_neighbors = [(x, y) for x, y in neighbors if (x, y) in self.grid_data[radar_id]]
        if valid_neighbors:
            values_sum = [self.grid_data[radar_id][neighbor] for neighbor in valid_neighbors]
            mean_value = (sum(values[0] for values in values_sum) / len(valid_neighbors),
                          sum(values[1] for values in values_sum) / len(valid_neighbors))
            self.grid_data[radar_id][grid_key] = mean_value

    def save_grid_data_to_file(self):

        for radar_id, grid_data in self.grid_data.items():
            print(str(grid_data.items()))
            file_name = f'{self.save_folder}/grid_data_{radar_id}.txt'
            print(f'Saving file: {file_name}')
            with open(file_name, 'w') as file:
                file.write(f'{self.map_width},{self.map_height},{self.grid_size}\n')
                for grid_key, value in grid_data.items():
                    file.write(f'{grid_key[0]},{grid_key[1]},{value[0]},{value[1]}\n')


if __name__ == '__main__':
    rospy.init_node('MultiRadarOffsetRecorder')

    rate = rospy.Rate(20)  # hz

    map_height = float(rospy.get_param('~map_height')) # Specify the map height in meters
    map_width = float(rospy.get_param('~map_width')) # Specify the map width in meters
    grid_size = float(rospy.get_param('~grid_size')) # Specify the grid size in meters
    window_size_in_ms = float(rospy.get_param('~window_size_in_ms')) # Specify the window size in milliseconds
    record_time_in_s = float(rospy.get_param('~record_time_in_s')) # Specify the total processing time in milliseconds


    radar_topics = json.loads(rospy.get_param('~radar_topics').replace('\'', '"'))
    radar_ids = json.loads(rospy.get_param('~radar_ids').replace('\'', '"'))
    save_folder = rospy.get_param('~save_folder')



    processor = RadarDataProcessor(window_size_in_ms, grid_size, map_width, map_height, save_folder)

    for index_radar in range(len(radar_topics)):
        radar_topic = radar_topics[index_radar]
        radar_id = radar_ids[index_radar]
        print(f'Listening to: {radar_id} in {radar_topic}')
        cloud_handler = lambda cloud, ri=radar_id: processor.callback_cloud(cloud,ri)
        rospy.Subscriber(radar_topic, PointCloud2, cloud_handler)

    start_time = rospy.get_rostime()
    while (rospy.get_rostime() - start_time).to_sec() < record_time_in_s:
        processor.loop()
        rate.sleep()

    print("Record timeout reached")

    processor.mean_corrections_by_grid()
    processor.interpolate_missing_values()
    processor.save_grid_data_to_file()

    print("Grid data saved successfully.")
