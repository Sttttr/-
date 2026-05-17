#include <pluginlib/class_list_macros.h>
#include "fds_planner/fds_planner.h"
#include <tf/tf.h>
#include <cmath>

// 注册插件，使其能被 move_base 动态加载
PLUGINLIB_EXPORT_CLASS(fds_planner::FDSPlanner, nav_core::BaseGlobalPlanner)

namespace fds_planner {

  // 默认构造函数
  FDSPlanner::FDSPlanner() : costmap_(NULL), initialized_(false) {}

  // 带参数的构造函数，直接调用初始化
  FDSPlanner::FDSPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
  }

  void FDSPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
      // 获取全局代价地图指针
      costmap_ = costmap_ros->getCostmap();
      initialized_ = true;
      ROS_INFO("FDS 全局规划器初始化成功！");
    }
  }

  /**
   * @brief 核心规划函数：实现 FDS 方向搜索算法
   */
  bool FDSPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
      ROS_ERROR("规划器尚未初始化，请检查配置。");
      return false;
    }

    plan.clear();
    
    // 算法参数设置
    double step_size = 0.15;      // 搜索步长 (单位: 米)
    double goal_tolerance = 0.3;  // 到达目标的容差
    double search_range = 6.28;    // 方向搜索范围 (弧度，约 60 度)
    int num_samples = 36;          // 扇区内的采样点数量

    geometry_msgs::PoseStamped current_pose = start;
    plan.push_back(current_pose);

    // 限制最大迭代次数，防止死循环
    int max_iterations = 3000;
    int iter = 0;

    while (iter < max_iterations) {
      // 1. 计算当前点到目标点的距离
      double dist_to_goal = hypot(goal.pose.position.x - current_pose.pose.position.x,
                                  goal.pose.position.y - current_pose.pose.position.y);
      
      // 如果接近目标，则结束搜索
      if (dist_to_goal < goal_tolerance) {
        break;
      }

      // 2. 计算指向目标的主方向角度
      double angle_to_goal = atan2(goal.pose.position.y - current_pose.pose.position.y,
                                   goal.pose.position.x - current_pose.pose.position.x);

      double best_cost = 255; // 初始设为最大代价
      geometry_msgs::PoseStamped best_next_pose = current_pose;
      bool found_valid_step = false;

      // 3. FDS 核心：在扇区内进行方向采样搜索
      for (int i = 0; i < num_samples; ++i) {
        // 在主方向左右分布采样角度
        double sample_angle = angle_to_goal + (i - num_samples / 2) * (search_range / num_samples);
        
        // 计算采样点的世界坐标
        double next_x = current_pose.pose.position.x + step_size * cos(sample_angle);
        double next_y = current_pose.pose.position.y + step_size * sin(sample_angle);

        // 转换到地图坐标并检查代价
        unsigned int mx, my;
        if (costmap_->worldToMap(next_x, next_y, mx, my)) {
          unsigned char cost = costmap_->getCost(mx, my);
          
          // // 如果不是障碍物且代价更低，则更新最佳点
          // if (cost < 253 && cost < best_cost) { 
          //   best_cost = cost;
          //   best_next_pose.pose.position.x = next_x;
          //   best_next_pose.pose.position.y = next_y;
          //   best_next_pose.header.frame_id = start.header.frame_id;
          //   // best_next_pose.header.stamp = ros::Time::now();
          //   best_next_pose.header.stamp = start.header.stamp; // 改成沿用 start 的安全时间戳
          //   found_valid_step = true;
          // 如果不是障碍物且代价更低 (注意这里加了 = 号，非常关键！防止等高代价死板选择偏向一侧)
          if (cost < 253 && cost <= best_cost) { 
            best_cost = cost;
            best_next_pose.pose.position.x = next_x;
            best_next_pose.pose.position.y = next_y;
            
            // 【极其关键的一步】：赋予该点一个随着朝向转动的合法四元数（姿态）！
            // 防止底层的 DWA 或 map-odom 监听因为读到非法的空四元数坐标而发生段错误（exit code -11）
            best_next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(sample_angle);

            best_next_pose.header.frame_id = start.header.frame_id;
            best_next_pose.header.stamp = ros::Time::now();
            found_valid_step = true;
          
          }
        }
      }

      // 4. 更新当前位置
      if (found_valid_step) {
        current_pose = best_next_pose;
        plan.push_back(current_pose);
      } else {
        ROS_WARN("FDS 搜索陷入死胡同，无法找到可行路径。");
        return false;
      }

      iter++;
    }

    // 将最终目标点加入路径
    plan.push_back(goal);
    ROS_INFO("FDS 规划成功：生成了 %zu 个路径点。", plan.size());

    return true;
  }
}