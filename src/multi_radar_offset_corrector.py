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
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import ros_numpy

class PointCloudCorrectionNode:
    def __init__(self, input_topic, output_topic, output_topic_all, save_folder, radar_id):
        self.grid_size = 0
        self.map_width = 0
        self.map_height = 0
        self.output_topic_all = output_topic_all
        self.grid_data = self.read_grid_data(save_folder, radar_id)

        self.output_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.output_all_pub = rospy.Publisher(output_topic_all, PointCloud2, queue_size=10)
        self.input_sub = rospy.Subscriber(input_topic, PointCloud2, self.pointcloud_callback)


    def read_grid_data(self, save_folder, radar_id):
        file_name = f'{save_folder}/grid_data_{radar_id}.txt'
        grid_data = {}

        with open(file_name, 'r') as file:
            header = True
            for line in file:
                if header==True:
                    header=False
                    width, height, grid_size = line.strip().split(',')
                    self.width = float(width)
                    self.height = float(height)
                    self.grid_size = float(grid_size)
                else:
                    grid_key_x, grid_key_y, value_x, value_y = line.strip().split(',')
                    grid_key = (int(grid_key_x), int(grid_key_y))
                    value = (float(value_x), float(value_y))
                    grid_data[grid_key] = value

        return grid_data

    def pointcloud_callback(self, data):
        points = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        # points = ros_numpy.point_cloud2.split_rgb_field(points)
        points_xyz = ros_numpy.point_cloud2.get_xyz_points(points, remove_nans=True)
        corrected_points = np.copy(points)
        for i, point in enumerate(points_xyz):
            x, y = point[:2]
            grid_indices = (np.floor(float(x) / self.grid_size).astype(int), np.floor(float(y) / self.grid_size).astype(int))
            print(f'grid_indices: {grid_indices}')
            if (grid_indices in self.grid_data):
                correction = self.grid_data[grid_indices[0], grid_indices[1]]
                print(f'Correction: {correction}')
                corrected_points[i]['x']-= correction[0]
                corrected_points[i]['y']-= correction[1]

        corrected_pc2 = ros_numpy.point_cloud2.array_to_pointcloud2(corrected_points, frame_id='map')
        self.output_pub.publish(corrected_pc2)
        self.output_all_pub.publish(corrected_pc2)

if __name__ == '__main__':
    rospy.init_node('MultiRadarOffsetCorrector')
    
    output_topic_all = rospy.get_param('~output_all_corrected')
    input_topic = rospy.get_param('~input_topic')
    output_topic = rospy.get_param('~output_topic')
    radar_id = rospy.get_param('~radar_id')
    save_folder = rospy.get_param('~save_folder')

    node = PointCloudCorrectionNode(input_topic, output_topic, output_topic_all, save_folder, radar_id)

    rospy.spin()