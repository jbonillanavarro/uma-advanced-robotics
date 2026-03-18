# Environment Perception & Geometric Feature Extraction

This section contains a **ROS 2 Humble** implementation of a perception and navigation node for a **Pioneer 3DX** differential drive robot. This practice was developed as part of the **Advanced Robotics** subject in the fourth year of the **Electronic, Robotic, and Mechatronic Engineering** degree at the **University of Málaga (UMA)**.

## Overview
Unlike the previous approach that relied on raw LiDAR point averaging to estimate clearances, this project focuses on extracting high-level geometric features from sensory data. The robot dynamically calculates the mathematical equations of the corridor walls using **Ordinary Least Squares (OLS) Linear Regression** and uses this geometric abstraction (distance and angle) to feed the path-tracking controller, ensuring a highly robust center-line trajectory.

*Note: The base node structure and ROS 2 topic subscriptions were provided by the UMA teaching staff. My specific contributions to this module include:*
1. **Geometric Feature Extraction:** Implementation of the `extractWall()` function to convert polar LiDAR data to Cartesian coordinates and fit straight lines ($y = mx + b$) representing the left and right walls.
2. **Robust Error Averaging & Control:** Update of the `laserCallback()` and `controlLoop()` to compute the lateral deviation and orientation error by averaging the geometric properties of both walls.

<img src="coppeliasim_simulation.gif" width="500" alt="Perception and Navigation Demo in CoppeliaSim">
