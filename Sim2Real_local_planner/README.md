# Sim2Real local planner
<!-- Note: add this repository to aurova_planning, but don't add it to the metapackage as it should not be installed -->
This repository has the code of the end-to-end local planner based on Imitation Learning of the paper "Sim-to-Real Transfer by Hybrid Gaussian Splatting and Geometric Reconstruction for Autonomous Driving". The network predicts the control action commands directly from the sensor information and the goal coordinates. The network is a local planner, and the localization and global planning are solved using our approach [Geo-Localization Based on Dynamically Weighted Factor-Graph](https://github.com/AUROVA-LAB/applications/tree/main/app_geo_navigation). 

## Installation
Clone [aurova_planning](https://github.com/AUROVA-LAB/aurova_planning) in your workspace. Compile the code to process the dataset and generate the safety matrices for training the model:
```
cd PATH/TO/CATKIN_WS/src/aurova_planning/Sim2Real_local_planner/process_data
mkdir build
cd build
cmake ..
make
```

Build the Docker image:
```
cd PATH/TO/CATKIN_WS/src/aurova_planning/Sim2Real_local_planner
docker build -t aurova_sim2real .
```

For using the neural network in the real robot, build the Docker image with ROS:
```
cd PATH/TO/CATKIN_WS/src/aurova_planning/Sim2Real_local_planner
docker build -f Dockerfile.ros -t aurova_sim2real:ros .
```

**Follow the instructions of app_Sim2Real for dataset generation, training, usage and evaluation.**