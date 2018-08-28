// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <occ_grid_mapping/grid_mapper.h>

GridMapper::GridMapper ( GridMap* map, Pose2d& T_r_l, double& P_occ, double& P_free, double& P_prior):
map_(map), T_r_l_(T_r_l), P_occ_(P_occ), P_free_(P_free), P_prior_(P_prior)
{
    
}

void GridMapper::updateMap ( const sensor_msgs::LaserScanConstPtr& scan,  Pose2d& robot_pose )
{
    /* 获取激光的信息 */
    const double& ang_min = scan->angle_min;
    const double& ang_max = scan->angle_max;
    const double& ang_inc = scan->angle_increment;
    const double& range_max = scan->range_max;
    const double& range_min = scan->range_min;
    
    /* 设置遍历的步长，沿着一条激光线遍历 */
    const double& cell_size = map_->getCellSize();
    const double inc_step = 1.0 * cell_size;

    /* for every laser beam */
    for(size_t i = 0; i < scan->ranges.size(); i ++)
    {
        /* 获取当前beam的距离 */
        double R = scan->ranges.at(i); 
        if(R > range_max || R < range_min)
            continue;
        
        /* 沿着激光射线以inc_step步进，更新地图*/
        double angle = ang_inc * i + ang_min;
        double cangle = cos(angle);
        double sangle = sin(angle);
        Eigen::Vector2d last_grid(Eigen::Infinity, Eigen::Infinity); //上一步更新的grid位置，防止重复更新
        for(double r = 0; r < R + cell_size; r += inc_step)
        {
            Eigen::Vector2d p_l(
                r * cangle,
                r * sangle
            ); //在激光雷达坐标系下的坐标
            
            /* 转换到世界坐标系下 */
            Pose2d laser_pose = robot_pose * T_r_l_;
            Eigen::Vector2d p_w = laser_pose * p_l;

            /* 更新这个grid */
            if(p_w == last_grid) //避免重复更新
                continue;
            
            updateGrid(p_w, laserInvModel(r, R, cell_size));
            	    
            last_grid = p_w;
        }//for each step
    }// for each beam
}

void GridMapper::updateGrid ( const Eigen::Vector2d& grid, const double& pmzx )
{
    /* TODO 这个过程写的太低效了 */
    double log_bel;
    if(  ! map_->getGridLogBel( grid(0), grid(1), log_bel )  ) //获取log的bel
        return;
    log_bel += log( pmzx / (1.0 - pmzx) ); //更新
    map_->setGridLogBel( grid(0), grid(1), log_bel  ); //设置回地图
}

double GridMapper::laserInvModel ( const double& r, const double& R, const double& cell_size )
{
    if(r < ( R - 0.5*cell_size) )
        return P_free_;
    
    if(r > ( R + 0.5*cell_size) )
        return P_prior_;
    
    return P_occ_;
}

