#ifndef FDS_PLANNER_H_
#define FDS_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace fds_planner {
  class FDSPlanner : public nav_core::BaseGlobalPlanner {
    public:
      FDSPlanner();
      FDSPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief 插件初始化函数
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief 核心规划函数：由 move_base 调用
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan);

    private:
      costmap_2d::Costmap2D* costmap_;
      bool initialized_;
      ros::Publisher plan_pub_; // 用于在 Rviz 中显示路径
  };
}

#endif