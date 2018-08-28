// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef GRID_MAPPER_H
#define GRID_MAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <occ_grid_mapping/grid_map.h>
#include <occ_grid_mapping/Pose2d.h>


class GridMapper{
public:
    GridMapper(GridMap* map,  Pose2d& T_r_l,  double& P_occ, double& P_free, double& P_prior);
    void updateMap(const sensor_msgs::LaserScanConstPtr& scan, Pose2d& robot_pose); //根据当前机器人的位姿和激光雷达数据跟新一次地图
    void updateGrid(const Eigen::Vector2d& grid, const double& pmzx);
    double laserInvModel(const double& r, const double& R, const double& cell_size);
    
private:
    GridMap* map_;
    Pose2d T_r_l_;
    double P_occ_, P_free_, P_prior_;
    
}; //class GridMapper

#endif

