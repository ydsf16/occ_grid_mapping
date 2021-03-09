# Occupancy Grid Mapping
## A simple implementation of occupancy grid mapping.

For details, please refer to：https://zhuanlan.zhihu.com/p/42995269

Dataset：https://pan.baidu.com/s/1j_SSEtaq7D0XwaED0Jg4Ew

## Demo

step1. Download the repository to your ROS workspace: catkin_ws/src

step2. Make：catkin_make

step3. Run: roslaunch occ_grid_mapping mapping.launch

step4. Palay a rosbag: rosbag play laser2_2018-07-14-18-41-42.bag -r 5

## Belief update

Set `bel` as belief of `grid_map(x, y)`, if (x, y) is in the range of global grid map.  
```c++
log_bel = log(bel) - log(1 - bel)
// where p is 0.5(unknown), 0.4(empty), 0.6(occupied)
// inverse model
log_bel += log(p) - log(1 - p)
bel = 1 - (1 + e^(log_bel))^(-1)
// grid_map(x, y) = bel // set back belief
```
The above update process can be simplied to 
```c++
odds = p/(1-p)*bel/(1 - bel)
bel = 1 - (1 + odds)^(-1)
// grid_map(x, y) = bel // set back belief
```

