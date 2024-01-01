# 招人啦！！！

Hello，小伙伴们！

我目前在蔚来汽车[NIO]自动驾驶部门工作，负责多传感器融合定位、SLAM等方面的研发工作。我们正在寻找新的同学加入。

目前，我们团队参与研发的智能驾驶功能已经在多个场景实现量产落地。高速城快领航辅助驾驶功能于2022年发布，累积服务里程超过1亿公里[截止10月]。今年蔚来开始向用户推送了技术难度更大的城区领航功能，近期正在通过群体智能方式不断拓展可用范围。蔚来独有的高速服务区领航体验也在今年11月份进行了发布，实现了高速到服务区换电场景的全流程自动化和全程领航体验。除了上面这些高阶自动驾驶功能，像AEB、LCC等背后也都有我们团队的身影。更多功能场景的发布，敬请期待。

**[全域领航辅助｜超3倍完成年度目标，提速规划节奏](app.nio.com/app/community_content_h5/module_10050/content?id=531584&type=article&is_nav_show=false&wv=lg)**

**现在，我们组正在寻找计算机视觉、深度学习、SLAM、多传感器融合、组合惯导等技术背景的同学加入，全职和实习均可以，欢迎来聊。微信: YDSF16**

<a href="https://youtu.be/3A5wpWgrHTI" target="_blank"><img src="https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/20231223-004022.jpeg" 
alt="DRE-SLAM" width="300" height="300"/></a>

NIO社招内推码: B89PQMZ 
投递链接: https://nio.jobs.feishu.cn/referral/m/position/detail/?token=MTsxNzAzMjY0NzE2NTYyOzY5ODI0NTE1OTI5OTgxOTI2NDg7NzI2MDc4NjA0ODI2Mjk2NTU0MQ

![image](https://github.com/ydsf16/TinyGrapeKit/blob/master/app/FilterFusion/doc/NIO-JD.jpeg)

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

