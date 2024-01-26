# README
This repository contains different ROS nodes related to the fusion of FMCW mmWave radar measurements from several radars:

- **target_follower**: uses a Kalman Filter to track a target.
- **multi_radar_to_single_topic**: Subscribes to measurements from different readers and republishes them in a common topic.
- **multi_positions_area_filter**: Removes measurements outside a certain area.

The ```launch``` folder contains some launch files to launch the algorithms with some parameters.

This repository is related with the next paper. Please cite us if this code is useful to you.

Barral, V., Dominguez-Bolano, T., Escudero, C. J., & Garcia-Naya, J. A. *An IoT System for Smart Building Combining Multiple mmWave FMCW Radars Applied to People Counting.*