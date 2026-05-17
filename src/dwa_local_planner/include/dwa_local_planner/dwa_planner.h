/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*

*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>

#include <dwa_local_planner/DWAPlannerConfig.h>

// 用于创建局部代价网格
#include <base_local_planner/map_grid_visualizer.h>

// 用于访问障碍物数据
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {

  /**
   * @class DWAPlanner
   * @brief 实现动态窗口法(Dynamic Window Approach)的局部路径规划器类
   */
  // ==========================================
  // 毕设创新点：FDS 方向诱导评价器 (Critic)
  // ==========================================
  class FdsDirectionCostFunction : public base_local_planner::TrajectoryCostFunction {
  public:
    FdsDirectionCostFunction() : target_yaw_(0.0) {}
    ~FdsDirectionCostFunction() {}

    bool prepare() { return true; }
     // 核心算法：评估采样轨迹末端朝向与 FDS 预期航向的偏差
    double scoreTrajectory(base_local_planner::Trajectory &traj) {
      if (traj.getPointsSize() == 0) return 0.0; // ⚠️关键修复：防止采样出空轨迹时导致底层端点提取崩溃

      double px, py, pth;
      traj.getEndpoint(px, py, pth); // 取出轨迹的最终位姿
      double diff = fabs(atan2(sin(pth - target_yaw_), cos(pth - target_yaw_)));
      return diff; 
    }
    //   // 核心算法：评估采样轨迹末端朝向与 FDS 预期航向的偏差
    //  double scoreTrajectory(base_local_planner::Trajectory &traj) {
    //     double px, py, pth;
    //     traj.getEndpoint(px, py, pth); // 取出轨迹的最终位姿
    //   // 计算角度差 (并限制在 -PI 到 PI 之间)
    //     double diff = fabs(atan2(sin(pth - target_yaw_), cos(pth - target_yaw_)));
    //     return diff;  // 框架会自动乘上我们在 CPP 里设置的 Weight 权重
    //       /* 核心算法：评估采样轨迹末端朝向与 FDS 预期航向的偏差 */ 
 
      

     void setTargetYaw(double yaw) { target_yaw_ = yaw; } 

  private:
    double target_yaw_;
  };
  class DWAPlanner {
    public:
      /**
       * @brief 构造函数
       * @param name 规划器名称
       * @param planner_util 规划器工具类的指针，用于辅助获取参数和数据
       */
      DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief 动态重新配置轨迹规划器参数
       */
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief 检查给定的位置/速度组合下，轨迹是否合法（即是否会发生碰撞）
       * @param pos 机器人当前位置
       * @param vel 机器人当前速度
       * @param vel_samples 期望的速度采样值
       * @return 如果轨迹安全/合法返回 true，否则返回 false
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief 根据机器人当前位置和速度，计算出得分最高（代价最低）的执行轨迹
       * @param global_pose 机器人当前位姿
       * @param global_vel 机器人当前速度
       * @param drive_velocities 输出参数：计算出的下一时刻驱动速度指令
       * @return 得分最高的轨迹。如果代价 >= 0，说明该轨迹合法可执行
       */
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          geometry_msgs::PoseStamped& drive_velocities);

      /**
       * @brief 在进行路径规划前，更新成本函数所需的数据
       * @param global_pose 机器人当前位姿
       * @param new_plan 最新的全局规划路径
       * @param footprint_spec 机器人的足迹（形状信息，用于避障计算）
       * * 注意：
       * 1. 障碍物代价函数使用 footprint。
       * 2. 路径和目标代价函数使用全局计划。
       * 3. 对齐代价函数使用基于当前位置修正后的全局计划。
       */
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan,
          const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief 获取局部规划器的仿真周期（即控制循环的频率）
       * @return 仿真时间步长（秒）
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief 计算地图网格单元的各项代价及总代价
       * @param cx, cy 网格的坐标
       * @param path_cost 输出：路径距离代价（偏离路径越远代价越高）
       * @param goal_cost 输出：目标距离代价（离目标越远代价越高）
       * @param occ_cost 输出：障碍物代价（单元格的占用情况）
       * @param total_cost 输出：加权后的总代价
       * @return 如果单元格可通行，返回 true
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * @brief 设置新计划并重置内部状态
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    private:
      base_local_planner::LocalPlannerUtil *planner_util_;

      // --- DWA 参数 ---
      double stop_time_buffer_;     ///< 缓冲区时间：在即将碰撞前，强制停止机器人的缓冲时间
      double path_distance_bias_;   ///< 路径距离权重：越强调该权重，机器人越紧贴全局路径
      double goal_distance_bias_;   ///< 目标距离权重：越强调该权重，机器人越趋向于向目标前进
      double occdist_scale_;        ///< 障碍物缩放系数：越高，避障意图越强
      Eigen::Vector3f vsamples_;    ///< 速度采样分辨率：在速度空间采样的样本数 (vx, vy, vtheta)
      double sim_period_;           ///< 仿真周期：计算 DWA 最大/最小速度的时间步长
      double forward_point_distance_; ///< 向前预测的距离：用于评估机器人姿态的角度
      double cheat_factor_;         ///< 偏差修正系数：用于微调代价计算的经验参数

      base_local_planner::Trajectory result_traj_; // 最终计算出的最优轨迹
      std::vector<geometry_msgs::PoseStamped> global_plan_; // 缓存的全局路径
      boost::mutex configuration_mutex_; // 多线程配置锁
      std::string frame_id_;

      // --- 可视化相关 ---
      ros::Publisher traj_cloud_pub_;
      bool publish_cost_grid_pc_;    ///< 是否构建并发布代价网格点云
      bool publish_traj_pc_;         ///< 是否发布预测的轨迹点云
      base_local_planner::MapGridVisualizer map_viz_; ///< 代价网格可视化工具，用于展示势场

      // --- 核心组件：轨迹生成与打分 ---
      // 1. 轨迹生成器：负责在速度空间模拟未来的轨迹
      base_local_planner::SimpleTrajectoryGenerator generator_;

      // 2. 代价函数（评委组）：对生成的轨迹进行多维度评分
      base_local_planner::OscillationCostFunction oscillation_costs_; // 震荡代价：避免左右摆动
      base_local_planner::ObstacleCostFunction obstacle_costs_;       // 障碍物代价：确保避障
      base_local_planner::MapGridCostFunction path_costs_;            // 路径代价：贴合路径
      FdsDirectionCostFunction fds_direction_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;            // 目标代价：靠近终点
      base_local_planner::MapGridCostFunction goal_front_costs_;      // 目标前向代价：鼓励车头朝向目标
      base_local_planner::MapGridCostFunction alignment_costs_;       // 对齐代价：轨迹与路径的方向对齐程度
      base_local_planner::TwirlingCostFunction twirling_costs_;       // 旋转代价：限制无意义的原地打转

      // 3. 采样规划器：统筹上述生成器和代价函数，选出最优解
      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
};
#endif