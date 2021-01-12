This repository is for paper *Efficient Local Trajectory Planning for High Speed Flight of a Quadrotor with Active Sensing*.
# Building Environment
- Ubuntu 16.04, ROS Kinetic
- Ubuntu 18.04, ROS Melodic
# Building Dependencies
- Eigen
- [Ewok](https://github.com/VladyslavUsenko/ewok/tree/master/ewok_ring_buffer/include/ewok). A version with our required IO functions is included in nags_planning/include.
# Test Results
## Simulation result
+ <center> Tabel I Simulation Test Results in Map (A) </center>
| Algorithm       | Avg. Flight Time | Collision Times | Trapped Times |
|-----------------|------------------|-----------------|---------------|
| U. Sampling [2] | 31.9s            | 0               | 0             |
| MAPF [3]        | xxx              | xxx             | xxx           |
| Our Planner     | 25.0s            | 0               | 0             |

+ <center> Tabel I Simulation Test Results in Map (B) </center>
| Algorithm       | Avg. Flight Time | Collision Times | Trapped Times |
|-----------------|------------------|-----------------|---------------|
| U. Sampling [2] | 44.9s            | 5               | 2             |
| MAPF [3]        | xxx              | xxx             | xxx           |
| Our Planner     | 42.7s            | 0               | 0             |

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
- New BSD License 

# Acknowledgement
