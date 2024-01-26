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


import string
from turtle import position
from attr import has
import numpy as np
from typing import Dict, List
import time
from scipy.spatial.distance import cdist
from geometry_msgs.msg import PointStamped, Point, PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Header
import rospy
import tf2_ros
import tf2_geometry_msgs
from kalman_filter import KalmanFilter
from gtec_msgs.msg import RadarFusedPointStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from kf_2d import KalmanFilter2D
from kf_2d_basic import KalmanFilter

class TargetPoint():
    def __init__(self, x: float, y:float, z:float, error_estimation:float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.error_estimation = error_estimation

class TargetFollower():
    def __init__(self, maxDistanceToAssignCentroidToTarget:float, windowTimeInMs:float, maxTargets:float, removeInactiveTargetTimeMs: float, loop_time:float, use_kf:bool) -> None:

        self.maxDistanceToAssignCentroidToTarget = maxDistanceToAssignCentroidToTarget
        self.windowTimeInMs = windowTimeInMs
        self.removeInactiveTargetTimeMs = removeInactiveTargetTimeMs

        self.window_positions_stack = []
        self.current_targets = {}
        self.last_window_time_ms = -1
        self.last_update_time_ms = -1
        self.all_targets_ids = list(range(maxTargets))
        self.loop_time = loop_time
        self.use_kf= use_kf

    
    def generateKF(self, dt, origin) -> KalmanFilter:
        # KF = KalmanFilter(dt=float(dt)/1000.0, measurementVariance=0.2, stateVariance=2, method="Velocity")
        # KF.predict()
        # KF.correct(np.matrix(origin).reshape(2,1), 10) #TODO:CHANGE THIS
        
        # KF2D
        # measurement_error = (0.15, 0.15) 
        # max_velocity = 0.5
        # KF = KalmanFilter2D(max_velocity, origin, measurement_error, float(dt)/1000.0)
        
        # KFBasic
        max_velocity = 0.4
        KF = KalmanFilter(max_velocity, float(dt)/1000.0, origin)

        return KF

    def addTargetsCloud(self, cloud: PointCloud2)-> None:
        
        #WE LET ONLY THEW LAST MSG
        self.window_positions_stack.clear()
        for p in pc2.read_points(cloud, field_names=("x", "y", "z", "error_est"), skip_nans=True):
            #print(f'P: {p}')
            self.addPosition(TargetPoint(p[0], p[1], p[2], p[3]))

    def addPosition(self, pos: TargetPoint)-> None:
        #print(f'New position: {pos.x},{pos.y}')
        #print(f'Recv position: {pos.x},{pos.y},{pos.z}. Error estimation: {pos.error_estimation}')
        self.window_positions_stack.append(pos)

    def getNewTargetId(self) -> int:
        if (len(self.all_targets_ids)>0):
            id = self.all_targets_ids[0]
            self.all_targets_ids.pop(0)
            return (id, True)
        return (0, False)

    def addNewTarget(self, position) -> None:
        (id, has_available_ids) = self.getNewTargetId()
        if (has_available_ids):
            kf = 0
            if (self.use_kf==True):
                kf = self.generateKF(self.loop_time, [position[0],position[1]])
            last_position_measurement = position
            start_live_time = 0
            cov_error = np.zeros([2,2])
            self.current_targets[id] = (position, start_live_time, kf, cov_error, last_position_measurement)

    def processWindow(self, elapsed_time) -> None:
        #num_points = len(self.window_positions_stack)
        array_coords = np.empty((0,3))
        coords_est_error = np.empty((0,1))
        for point in self.window_positions_stack:
            array_coords = np.vstack([array_coords, [point.x, point.y, point.z]])
            coords_est_error = np.vstack([coords_est_error, [point.error_estimation]])
            
        
        self.window_positions_stack.clear()

        # if (len(array_coords)<1):
        #     return
        
        #From here, there are at least one target

        # If empty, we add the first target
        if len(self.current_targets.keys())==0 and len(array_coords)>0:
            self.addNewTarget(array_coords[0])


        # We need to check is some of the new centroids are near the previous ones
        targets_ids = list(self.current_targets.keys())
        print(f'Targetids # {len(targets_ids)}')
        # We increase the last seen time of each current target
        for target_id in targets_ids:
            target_pos = self.current_targets[target_id][0]
            target_time_alive = self.current_targets[target_id][1]
            target_kf = self.current_targets[target_id][2]
            target_cov_error = self.current_targets[target_id][3]
            target_last_position_measurement = self.current_targets[target_id][4]
            new_time = target_time_alive + elapsed_time
            self.current_targets[target_id] = (target_pos , new_time, target_kf, target_cov_error, target_last_position_measurement)
            print(f'Target {target_id} pos = {target_pos}')


        # self.current targets has HERE the list of targets previously detected.

        positions_by_target_id = {}
        centroids_assigned = []
        targets_assigned = []

        # #FIRST we have to reduce the new positions if they are very close

        # if (len(array_coords)>0):
        #     #CLUSTERING HERE

        #     # O SI NO HACER EL CLUSTERING FUERA CON OTRO NODO


        # We check if the new positions are near of some of the previous targets
        if (len(array_coords)>0):
            current_pos = [i[0] for i in self.current_targets.values()]
            print(f'New pos to compare: {array_coords}')
            #distance_to_previous_targets = cdist( current_pos, array_coords, metric='euclidean')
            distance_to_previous_targets = cdist( current_pos, array_coords, lambda u, v: np.sqrt(((u[1:2]-v[1:2])**2).sum()))
            print(f'distance_to_previous_targets: {distance_to_previous_targets}')
                
            # Now we are picking the min distance of the matrix to assign new centroids to old targets
            # After one assignation, we remove the posibility of further associations using the same target and centroids
            # At the end, old targets without nearby centroids are removed, and new centroids without nearby targets are promoted
            # to new targets.
            


            masked_distances = np.ma.masked_greater(distance_to_previous_targets, self.maxDistanceToAssignCentroidToTarget)
            print(f'Masked distances: {masked_distances} Mask: {masked_distances.mask}')
            
            if (len(masked_distances)==1):
                masked_distances.mask = [masked_distances.mask]
            
            
            
            
            can_end = False

            while not can_end:
                #print(f'While not can_end')
                if (np.all(masked_distances.mask == True)):
                    can_end = True  
                    #print(f'Can _end= True')
                else:
                    #print(f'Can _end= False')
                    # We assign the previous target to a neraby centroid
                    min_distance_cell = np.unravel_index(masked_distances.argmin(), masked_distances.shape)
                    target_id = targets_ids[min_distance_cell[0]]
                    centroid_pos = array_coords[min_distance_cell[1]]
                    centroid_pos_error_estimation = coords_est_error[min_distance_cell[1]]
                    centroids_assigned.append(min_distance_cell[1])
                    targets_assigned.append(target_id)

                    # We also set the last_view time of the target to 0
                    if (self.use_kf==True):
                        #print(f'Using KF:')
                        kf = self.current_targets[target_id][2]
                        # kf.correct(np.matrix(np.array([centroid_pos[0], centroid_pos[1]])).reshape(2,1), 1.5) #TODO:CHANGE THIS TO ACTUAL VALUE
                        # #kf.update(np.matrix(np.array([centroid_pos[0], centroid_pos[1]])).reshape(2,1))
                        # (pred, cov_error_xy) = kf.predict()
                        # prediction= np.array(pred).reshape(1,2)
                        # new_pos = np.array([prediction[0][0], prediction[0][1], centroid_pos[2]])

                        # KF 2d
                        # kf.predict()
                        # new_pos_x, new_pos_y, cov_error_xy = kf.update((centroid_pos[0], centroid_pos[1]))
                        # new_pos = np.array([new_pos_x, new_pos_y,centroid_pos[2]])
                        
                        # KF Simple
                        pos_error = centroid_pos_error_estimation[0]
                        new_pos_x, new_pos_y, cov_error_xy = kf.update((centroid_pos[0], centroid_pos[1], pos_error))
                        new_pos = np.array([new_pos_x, new_pos_y, centroid_pos[2]])
                        last_measurement_pos = np.array([centroid_pos[0], centroid_pos[1], centroid_pos[2]])
                        
                        #print(f'New prediction: {new_pos}')
                        positions_by_target_id[target_id] = (new_pos, 0, kf, cov_error_xy, last_measurement_pos)
                    else:
                        #print(f'NOT Using KF:')
                        new_pos = np.array([centroid_pos[0], centroid_pos[1], centroid_pos[2]])
                        positions_by_target_id[target_id] = (new_pos, 0, 0, 0, new_pos)

                    #prediction = np.array(kf.predict()).reshape(1,2)
                    #new_pos = np.array([prediction[0][0], prediction[0][1]])

                    #positions_by_target_id[target_id] = (new_pos, 0, kf)
                    #print(f'Target assigned: {target_id}')
                    #print(f'Mask: {masked_distances}')
                    
                    try:
                        # We remove the posibility of assign again the same target or centroid
                        if (len(masked_distances)==1):
                            print(f'len(masked_distances) == 1')
                            masked_distances.mask[:,:]= True
                            #can_end = True
                        else:
                            print(f'len(masked_distances) != 1')
                            # print(f'min_distance_cell[0]= {min_distance_cell[0]}, min_distance_cell[1]= {min_distance_cell[1]}')
                            masked_distances.mask[min_distance_cell[0],:] = True
                            # print(f'Mask: {masked_distances}')
                            masked_distances.mask[:,min_distance_cell[1]] = True
                            #print(f'Mask: {masked_distances}')
                    except Exception as e:
                        print(e)
                        can_end = True
                        return

        #print(f'Previous targets with new positions near: {targets_assigned}')
        # There are no more valid distances
        targets_not_assigned = [x for x in targets_ids if x not in targets_assigned]
        #print(f'Old targets with no new position: {targets_not_assigned}')
        # We 'free' the ids of the targets only if passed enough time
        for target_not_assigned_id in targets_not_assigned:
            #print(f'Working target not assigned{target_assigned_id}. Time: {self.current_targets[target_assigned_id][1]}')
            if (self.current_targets[target_not_assigned_id][1]>=self.removeInactiveTargetTimeMs):
                # We can free the id to be reused
                self.all_targets_ids.append(target_not_assigned_id)
            else:
                # We use the KF to predict a new position
                if (self.use_kf== True):
                    current_pos = self.current_targets[target_not_assigned_id][0]
                    last_measurement_pos = self.current_targets[target_not_assigned_id][4]
                    kf = self.current_targets[target_not_assigned_id][2]
                    
                    new_pos_x, new_pos_y, cov_error_xy = kf.update((last_measurement_pos[0], last_measurement_pos[1], 0.5))
                    new_pos = np.array([new_pos_x, new_pos_y,current_pos[2]])
                    
                    
                    # kf.predict()
                    # new_pos_x, new_pos_y, cov_error_xy = kf.update((current_pos[0], current_pos[1]))
                    # new_pos = np.array([new_pos_x, new_pos_y,current_pos[2]])
                    
                    
                    # kf.correct(np.matrix(np.array([current_pos[0], current_pos[1]])).reshape(2,1), 1.5) #TODO:CHANGE THIS TO ACTUAL VALUE
                    # #kf.update(np.matrix(np.array([centroid_pos[0], centroid_pos[1]])).reshape(2,1))
                    # (pred, cov_error_xy) = kf.predict()
                    # prediction= np.array(pred).reshape(1,2)
                    # new_pos = np.array([prediction[0][0], prediction[0][1], current_pos[2]])
                    self.current_targets[target_not_assigned_id] = (new_pos, self.current_targets[target_not_assigned_id][1] , kf, cov_error_xy, last_measurement_pos)
                positions_by_target_id[target_not_assigned_id] = self.current_targets[target_not_assigned_id]

        #self.all_targets_ids.extend(targets_not_assigned)
        # We check if some centroid remains unasigned
        all_centroids_indexes = range(len(array_coords))
        centroids_not_assigned = [x for x in all_centroids_indexes if x not in centroids_assigned]
        for centroid_index in centroids_not_assigned:
            (new_target_id, has_available_ids) = self.getNewTargetId()
            if (has_available_ids):
                kf=0
                if (self.use_kf==True):
                    kf = self.generateKF(self.loop_time, [array_coords[centroid_index][0], array_coords[centroid_index][1]] )
                last_measurement_pos = np.array([array_coords[centroid_index][0], array_coords[centroid_index][1], array_coords[centroid_index][2]])
                positions_by_target_id[new_target_id] = (array_coords[centroid_index],0, kf, np.zeros([2,2]), last_measurement_pos)

        self.current_targets = positions_by_target_id
    
    #print(f'self.current_targets = {self.current_targets}')
            

    def loop(self) -> Dict:
        
        current_time_ms = float(rospy.get_rostime().to_nsec())/1000000.0
        if (current_time_ms>0):
            if (self.last_window_time_ms<0):
                self.last_window_time_ms = current_time_ms
            
            elapsed_in_ms = current_time_ms - self.last_window_time_ms

            if elapsed_in_ms>=self.windowTimeInMs:
                self.processWindow(elapsed_in_ms)
                self.last_window_time_ms = current_time_ms

        return self.current_targets


if __name__ == "__main__":

    rospy.init_node('TargetFollower', anonymous=True)
    rate = rospy.Rate(20)  # hz

    target_positions_topic = rospy.get_param('~all_targets_topic')
    publish_targets_topic = rospy.get_param('~publish_target_topic')
    publish_targets_with_covariance_topic = rospy.get_param('~publish_target_with_covariance_topic')

    maxTargets = int(rospy.get_param('~max_targets', 10))
    maxDistanceToAssignCentroidToTarget = float(rospy.get_param('~max_distance_to_assign_centroid_to_target',1))
    windowTimeInMs = int(rospy.get_param('~window_time_in_ms',500))
    removeInactiveTargetTimeMs = float(rospy.get_param('~remove_inactive_target_time_ms',5000))

    use_kf = rospy.get_param('~use_kf')

    target_follower = TargetFollower(
    maxDistanceToAssignCentroidToTarget=maxDistanceToAssignCentroidToTarget,
    windowTimeInMs=windowTimeInMs, 
    maxTargets=maxTargets, 
    removeInactiveTargetTimeMs=removeInactiveTargetTimeMs,
    loop_time=windowTimeInMs,
    use_kf=use_kf)

    print(f'UseKF: {use_kf}')

    pub_fused_targets= rospy.Publisher(publish_targets_topic, RadarFusedPointStamped, queue_size=100)
    pub_fused_targets_cloud= rospy.Publisher(publish_targets_topic+'/cloud', PointCloud2, queue_size=100)

    rospy.Subscriber(str(target_positions_topic), PointCloud2, target_follower.addTargetsCloud)  

    print("=========== GTEC mmWave Target Follower Node ============")
    
    last_num_targets = -1
    
    while not rospy.is_shutdown():
        current_targets = target_follower.loop()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        targets_cloud_points = []
        if len(current_targets)!= last_num_targets:
            last_num_targets = len(current_targets)
            print(f"Num targets: {last_num_targets}")
            
        
        for target_id in current_targets.keys():
            pos = current_targets[target_id][0]
            point = Point(pos[0], pos[1], pos[2])
            targets_cloud_points.append([pos[0], pos[1], pos[2]])
            radarFusedPointStamped = RadarFusedPointStamped()
            radarFusedPointStamped.header = header
            radarFusedPointStamped.point = point
            radarFusedPointStamped.targetId = target_id
            pub_fused_targets.publish(radarFusedPointStamped)

        targets_fields_cloud = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        targets_cloud = pc2.create_cloud(
            header, targets_fields_cloud, targets_cloud_points)    
        pub_fused_targets_cloud.publish(targets_cloud)

        rate.sleep()