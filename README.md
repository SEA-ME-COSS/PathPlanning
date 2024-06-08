# About

This repository corresponds to the Path Planning part of an Autonomous Driving System. It uses ROS2 to subscribe to various information from perception and publishes the path and velocity to be followed. Based on the information received from perception, it makes decisions and determines the velocity accordingly. Then, it uses the hybrid A* developed in ROS2 to create the required path. If you're interested in a detailed usage of these algorithms, visit the following repository to test them out.

# Requirements

- Ubuntu 20.04
- [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- matplotlib & numpy
    
    ```bash
    pip install numpy
    pip install matplotlib
    ```
    
- Eigen
    
    ```bash
    sudo apt update	
    sudo apt install libeigen3-dev
    ```
    
- Ceres
    
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev libgoogle-glog-dev libgflags-dev
    sudo apt install libceres-dev
    ```
    
- ompl
    
    ```bash
    sudo apt install libompl-dev
    ```
    

# Usage

If you want to test the algorithms without any additional installations, use the following commands:

```bash
git clone https://github.com/SEA-ME-COSS/PathPlanning.git
cd PathPlanning/algorithm
mkdir build && cd build
cmake ..
make
./path_planning
```

# Reference

- [Hybrid A Star](https://github.com/bmegli/hybrid-a-star)
- [ROS2 foxy](https://docs.ros.org/en/foxy/index.html)