#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>
#include <queue>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace dwa_local_planner {

  /**
   * @brief 动态参数重配置：当你在 Rviz 或参数服务器修改参数时，此函数会被调用
   */
  void DWAPlanner::reconfigure(DWAPlannerConfig &config)
  {
    // 使用互斥锁，防止在修改参数的同时进行路径规划导致程序崩溃
    boost::mutex::scoped_lock l(configuration_mutex_);

    // 设置轨迹生成器的参数：仿真时间、步长、是否使用 DWA 模式等
    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    // 根据地图分辨率缩放代价权重，确保在不同分辨率的地图下表现一致
    double resolution = planner_util_->getCostmap()->getResolution();
    
    // 路径距离权重：决定机器人有多“听话”（贴近全局路径）
    path_distance_bias_ = resolution * config.path_distance_bias;
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);

    // 目标距离权重：决定机器人有多“心急”（直奔局部目标点）
    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    // 障碍物权重系数
    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    // 停止缓冲区、震荡重置距离等参数设置
    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    
    // 设置“前瞻点”距离：用于计算机器人车头（而非中心）的对齐代价
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    // 设置避障参数：最大平移速度、足迹缩放因子等
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    // 设置旋转惩罚权重：防止无意义的原地打转
    twirling_costs_.setScale(config.twirling_scale);

    // 速度采样数设置：采样越多，搜索越精细，但越耗 CPU
    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
 
    // 安全检查：至少要采样一个值
    if (vx_samp <= 0) { vx_samp = 1; config.vx_samples = vx_samp; }
    if (vy_samp <= 0) { vy_samp = 1; config.vy_samples = vy_samp; }
    if (vth_samp <= 0) { vth_samp = 1; config.vth_samples = vth_samp; }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
  }

  /**
   * @brief 构造函数：初始化各个评价器（Critics）和可视化工具
   */
  DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    ros::NodeHandle private_nh("~/" + name);

    // 默认不因为某个单元格不可达而停止整个评价过程
    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    // 自动寻找控制频率（通常在 move_base 中设置）
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05; // 默认 20Hz
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      sim_period_ = (controller_frequency > 0) ? (1.0 / controller_frequency) : 0.05;
    }

    oscillation_costs_.resetOscillationFlags();

    // 设置是否累加网格得分
    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);

    // 初始化可视化工具：可以在 Rviz 中看到代价网格的颜色
    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name,
                        planner_util->getGlobalFrame(),
                        [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost){
                          return getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
                        });

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));
    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // --- 核心环节：注册评价器（Critics） ---
    // 评价器的顺序会影响计算效率。返回负值的评价器会直接剔除该轨迹。
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // 1. 剔除震荡轨迹
    critics.push_back(&obstacle_costs_);    // 2. 剔除撞墙轨迹
    critics.push_back(&goal_front_costs_);  // 3. 鼓励车头朝向局部目标
    critics.push_back(&alignment_costs_);   // 4. 鼓励车头贴合路径
    
    // --- 毕设新增：FDS 方向评价器 ---
    fds_direction_costs_.setScale(5.0); // 设置较高的权重，让机器人更倾向于保持特定航向
    critics.push_back(&fds_direction_costs_);

    critics.push_back(&path_costs_); // 5. 贴合全局路径
    critics.push_back(&goal_costs_); // 6. 奔向局部目标点
    critics.push_back(&twirling_costs_); // 7. 惩罚自旋

    // 初始化采样规划器
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);
    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
  }

  /**
   * @brief 计算并返回单个地图单元格的各项代价（主要用于调试和可视化）
   */
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);

    // 如果该点本身是障碍物或不可达，返回 false
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    // 计算加权总得分
    total_cost = path_distance_bias_ * path_cost +
                 goal_distance_bias_ * goal_cost +
                 occdist_scale_ * occ_cost;
    return true;
  }

  /**
   * @brief 设置全局路径，并重置震荡标志位
   */
  bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * @brief 单独检查某条特定轨迹的合法性
   */
  bool DWAPlanner::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    
    generator_.initialise(pos, vel, goal, &limits, vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    if(cost >= 0) return true;

    ROS_WARN("无效轨迹速度: %.2f, %.2f, %.2f, 代价: %.2f", vel_samples[0], vel_samples[1], vel_samples[2], cost);
    return false;
  }

  /**
   * @brief 更新局部规划所需的各种数据
   */
  void DWAPlanner::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }
     // ======【增加开始】防崩溃保护 =======
    if (global_plan_.empty()) {
      ROS_WARN("接收到的全局路径为空，跳过本次更新。");
      return; 
    }

    obstacle_costs_.setFootprint(footprint_spec);
    path_costs_.setTargetPoses(global_plan_);
    goal_costs_.setTargetPoses(global_plan_);

    // 计算当前位置与最终目标的距离
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist = (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
                     (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // 计算“虚拟车头”位置，这有助于机器人在转弯时更自然
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x += forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y += forward_point_distance_ * sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    
    // 【修改】：无条件设置目标路径，以防当离目标近进入 else 分支时由于未设置 target_poses 导致底层 MapGrid 报错
    alignment_costs_.setTargetPoses(global_plan_);

    // 如果还没到终点，应用对齐代价；快到终点时关闭对齐，防止原地晃动
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      alignment_costs_.setScale(path_distance_bias_);
    } else {
      alignment_costs_.setScale(0.0);
    }

    // --- 毕设新增：提取 FDS 全局路径前方点，计算期望航向角 ---
    if (!new_plan.empty()) {
      // 选取前方第 10 个点作为目标方向点，这比取第 1 个点更平滑
      int target_idx = std::min((int)new_plan.size() - 1, 10);
      double dx = new_plan[target_idx].pose.position.x - new_plan[0].pose.position.x;
      double dy = new_plan[target_idx].pose.position.y - new_plan[0].pose.position.y;
      double fds_yaw = atan2(dy, dx); // 计算路径切线方向
      fds_direction_costs_.setTargetYaw(fds_yaw);
    }
  }

  /**
   * @brief 寻找最优路径：算法的核心主循环
   */
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities) {

    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // 1. 初始化生成器
    generator_.initialise(pos, vel, goal, &limits, vsamples_);

    // 2. 调用采样规划器，在速度空间搜索得分最高的轨迹
    result_traj_.cost_ = -7; // 初始负值表示尚未找到
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    // 3. (可选) 可视化采样的轨迹云
    if(publish_traj_pc_) {
        // ... 此处为构建点云并在 Rviz 中显示的逻辑 ...
        // (保持原样，主要是为了在 Rviz 看到那一团彩色线条)
    }

    // 4. 可视化代价网格
    if (publish_cost_grid_pc_) {
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // 5. 根据选出的最优轨迹更新震荡标志
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    // 6. 结果输出：如果没有合法轨迹，命令机器人停下；否则发送计算出的速度
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.orientation.w = 1;
    } else {
      drive_velocities.pose.position.x = result_traj_.xv_;
      drive_velocities.pose.position.y = result_traj_.yv_;
      tf2::Quaternion q;
      q.setRPY(0, 0, result_traj_.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return result_traj_;
  }
};