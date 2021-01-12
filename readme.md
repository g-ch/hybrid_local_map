This repository is for paper *Efficient Local Trajectory Planning for High Speed Flight of a Quadrotor with Active Sensing*.
# Building Environment
- Ubuntu 16.04, ROS Kinetic
- Ubuntu 18.04, ROS Melodic
# Building Dependencies
- Eigen
- [Ewok](https://github.com/VladyslavUsenko/ewok/tree/master/ewok_ring_buffer/include/ewok). A version with our required IO functions is included in nags_planning/include.
# Building Steps
Clone this repository to your ROS workspace and run
```
catkin_make
```
# Test Results
## Simulation Result
- Tabel I Simulation Test Results in Map (A)

| Algorithm       | Avg. Flight Time | Collision Times | Trapped Times |
|-----------------|------------------|-----------------|---------------|
| U. Sampling [2] | 31.9s            | 0               | 0             |
| MAPF [3]        | xxx              | xxx             | xxx           |
| Our Planner     | 25.0s            | 0               | 0             |

- Tabel II Simulation Test Results in Map (B) 

| Algorithm       | Avg. Flight Time | Collision Times | Trapped Times |
|-----------------|------------------|-----------------|---------------|
| U. Sampling [2] | 44.9s            | 5               | 2             |
| MAPF [3]        | xxx              | xxx             | xxx           |
| Our Planner     | 42.7s            | 0               | 0             |

## Realworld Tests
- Qaudrotor Specification

| Rotor Base | Propeller Size | Motor           | Weight | Flight Controller | Onboard Computer |
|------------|----------------|-----------------|--------|-------------------|------------------|
| 210 mm     | 5 inches       | T-motor F40 Pro | 1.2 kg | Pixracer          | Up core board    |

- Sensor Specification

| Sensor          | Property                                                             | Usage                                                            |
|-----------------|----------------------------------------------------------------------|------------------------------------------------------------------|
| Realsense D435  | Max Range: about 10m FOV: 86°x 57°                                   | To generate pointclouds at 30Hz. Depth image resolution: 424x240 |
| Realsense T265  | Providing under 1% closed  loop drift under intended use conditions. | To provide state estimation of the quadrotor.                    |
| FT motor SM30BL | Angle resolution: 0.088° Max torque: 10kg*com                        | Control the active sensing camera and measure its angle.         |

- Test video
See:
<https://youtu.be/zgIZDW39KJs>

# Citation
G. Chen, D. Sun, W. Dong, X. Sheng, X. Zhu, and H. Ding, "Efficient Local Trajectory Planning for High Speed Flight of a Quadrotor with Active Sensing", 2021. (Under revision)

```
@article{ActiveSensing,
title={Bio-inspired Obstacle Avoidance for Flying Robots with Active Sensing},
author={Chen, Gang and Dong, Wei and Sheng, Xinjun and Zhu, Xiangyang and Ding, Han},
journal={arXiv preprint arXiv:2010.04977},
year={2020}}
```

# License
+ New BSD License 

# Acknowledgement
We thank Shuhan He and Boyu Zhou for their help on this work.
