#!/usr/bin/env python3

""" MIT License

Copyright (c) 2023 Group of Electronic Technology and Communications. University of A Coruna.

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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('MultiRadarToSingleTopic')

    rate = rospy.Rate(20)  # hz


    radar_topics = json.loads(rospy.get_param('~radar_topics').replace('\'', '"'))
    output_topic = rospy.get_param('~output_topic')

    output_pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)


    for radar_topic in radar_topics:
        msg_handler = lambda msg: output_pub.publish(msg)
        rospy.Subscriber(radar_topic, PointCloud2, msg_handler)
        
    rospy.spin()

