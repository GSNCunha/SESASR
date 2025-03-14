# Service Robotics Laboratory Projects

This repository contains the projects developed for the "Sensors, Embedded Systems, and Algorithms for Service Robotics" course at Politecnico di Torino (POLITO). The course included six laboratory assignments, each focused on different aspects of service robotics. Inside the folder for each lab, you’ll find the task descriptions, and for the last three labs, the reports detailing the work done.

## Project Overview

The main challenges involved interacting with a robot both in simulation and physically, using nodes, topics, and subscriptions in ROS 2 with Python. Various algorithms were implemented to enable the robot to solve the proposed problems efficiently:

- **Extended Kalman Filter (EKF):** A state estimation algorithm used to track the robot’s position and orientation by combining sensor data (like odometry and IMU) while handling non-linearities. It predicts the robot's state and corrects it based on incoming measurements, improving localization accuracy.

- **Dynamic Window Approach (DWA):** A local path planning algorithm used to navigate the robot while avoiding obstacles. It evaluates possible velocities and trajectories, choosing the one that balances speed, safety, and goal direction.

- **Particle Filter:** A probabilistic localization algorithm that represents the robot's possible positions using a set of particles. Each particle’s weight is adjusted based on sensor readings, and the robot’s most likely position is inferred from the distribution of particles.

## Tools and Technologies

- **ROS 2:** The Robot Operating System, used for creating and managing distributed systems with nodes and topics.
- **Python:** The main programming language for developing algorithms and node communications.

## Reports

Reports for the last three labs provide a detailed explanation of the approach, implementation, and results for each task.
