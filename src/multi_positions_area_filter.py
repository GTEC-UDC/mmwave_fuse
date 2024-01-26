#!/usr/bin/env python

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

from geometry_msgs.msg import PointStamped, Point, Polygon
from std_msgs.msg import Header
import rospy
import tf2_ros
import tf2_geometry_msgs
import cmath
import json


class MapFilter():
    def __init__(self, map_polygon: Polygon) -> None:
        self.map_polygon = map_polygon

    def filter(self, point:Point) -> bool:
        return self.isInsidePolygon((point.x, point.y), self.map_polygon.points, True)

    def is_P_InSegment_P0P1(self, P, P0, P1):
        p0 = P0[0]- P[0], P0[1]- P[1]
        p1 = P1[0]- P[0], P1[1]- P[1]
        det = (p0[0]*p1[1] - p1[0]*p0[1])
        prod = (p0[0]*p1[0] + p0[1]*p1[1])
        return (det == 0 and prod < 0) or (p0[0] == 0 and p0[1] == 0) or (p1[0] == 0 and p1[1] == 0)   

    def isInsidePolygon(self, P: tuple, Vertices: list, validBorder=False) -> bool:
        sum_ = complex(0,0)

        for i in range(1, len(Vertices) + 1):
            v0, v1 = Vertices[i-1] , Vertices[i%len(Vertices)]
            if self.is_P_InSegment_P0P1(P,(v0.x, v0.y), (v1.x, v1.y)):
                return validBorder
            sum_ += cmath.log( (complex(*(v1.x, v1.y)) - complex(*P)) / (complex(*(v0.x, v0.y)) - complex(*P)) )
        return abs(sum_) > 1


if __name__ == "__main__":
    rospy.init_node('MultiPositionAreaFilter', anonymous=True)
    rate = rospy.Rate(10)  # hz

    positions_topic = rospy.get_param('~multi_positions_topic')
    publish_topic = rospy.get_param('~publish_topic')
    polygons_file = rospy.get_param('~polygons_file')

    pub = rospy.Publisher(str(publish_topic), PointStamped, queue_size=100)

    polygons_json = json.loads(str(polygons_file))
    map_filters = []

    for poly in polygons_json["polygons"]:
        pol = Polygon()
        pol.points = []
        for a_point in poly["points"]:
            pol.points.append(Point(a_point["x"], a_point["y"], a_point["z"]))
        map_filters.append(MapFilter(pol))

    tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0)) 
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    def getTransformedPoint(point_msg):
        transform_radar_to_odom = tf_buffer.lookup_transform("odom",
                                point_msg.header.frame_id, 
                                rospy.Time(0), 
                                rospy.Duration(1.0))
        tf_pos = tf2_geometry_msgs.do_transform_point(point_msg, transform_radar_to_odom)
        return (point_msg.header, Point(tf_pos.point.x, tf_pos.point.y, tf_pos.point.z))

    def publishPositionIfInside(header_and_pos):
        (header, pos) = header_and_pos
        inside = False
        for map_filter in map_filters:
            if (map_filter.filter(pos)):
                inside = True
                break

        if inside:
            pub.publish(PointStamped(Header(seq=header.seq, stamp=header.stamp, frame_id="odom"), pos))

    pos_handler = lambda pos: publishPositionIfInside(getTransformedPoint(pos))
    rospy.Subscriber(str(positions_topic), PointStamped, pos_handler) 

    while not rospy.is_shutdown():
        rate.sleep()