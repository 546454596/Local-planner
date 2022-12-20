#include "timed_elastic_band/teb_local_planner.h"
namespace roborts_local_planner {

TebLocalPlanner::TebLocalPlanner () { }

TebLocalPlanner::~TebLocalPlanner () { }
//在local_planner节点中先执行1.SetPlan  2.Initialize  3.ComputeVelocityCommand  4.IsGoalReached
//在SetPlan获取了temp_plan_（应该为局部规划坐标系下路径）然后，.2.完成优化算法参数设置 3.首先GetPlan(temp_plan_),获得global_plan_（局部规划坐标系下轨迹点）；
roborts_common::ErrorInfo TebLocalPlanner::ComputeVelocityCommands(roborts_msgs::TwistAccel &cmd_vel) {
  if (!is_initialized_) {
    roborts_common::ErrorInfo algorithm_init_error(roborts_common::LP_ALGORITHM_INITILIZATION_ERROR,"teb initialize failed");
    ROS_ERROR("%s",algorithm_init_error.error_msg().c_str());
    return algorithm_init_error;
  }
  GetPlan(temp_plan_);//获取局部规划坐标系下的轨迹点，如果path为空，则放入goal点
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;//进行初始化赋值
  cmd_vel.accel.linear.x = cmd_vel.accel.linear.y = cmd_vel.accel.angular.z = 0;
  UpdateRobotPose();//获取机器人 全局坐标系下 位姿 robot_pose_
  UpdateRobotVel();//base_link的速度
  UpdateGlobalToPlanTranform();//由局部规划到全局坐标系之间的变换
  auto time_now = std::chrono::system_clock::now();
  oscillation_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - oscillation_).count() / 1000.0f;
  if (oscillation_time_ > 1.0) {
    if ((robot_pose_.GetPosition() - last_robot_pose_.GetPosition()).norm() < 0.1) {
      local_cost_.lock()->ClearCostMap();
    } else {
      oscillation_time_ = 0;
      oscillation_ = std::chrono::system_clock::now();
      last_robot_pose_ = robot_pose_;
    }
  }
  if (IsGoalReached()) {
    roborts_common::ErrorInfo algorithm_ok(roborts_common::OK, "reached the goal");
    return algorithm_ok;
  }
  PruneGlobalPlan();//留下距离机器人后0.8米到最终的目的地之间的局部规划轨迹点
  int goal_idx;//goal_idx为符合要求的i-1，表示局部规划的在全局坐标系下的局部目的地,通过  TransformGlobalPlan   获得，且获得  transformed_plan_
  if (!TransformGlobalPlan(&goal_idx)) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "plan transform error");
    ROS_ERROR("%s", PlanTransformError.error_msg().c_str());
    return PlanTransformError;
  }
  if (transformed_plan_.empty()) {
    roborts_common::ErrorInfo PlanTransformError(roborts_common::LP_PLANTRANSFORM_ERROR, "transformed plan is empty");
    ROS_ERROR("transformed plan is empty");
    return PlanTransformError;
  }
  /*tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan_.poses.back(), goal_point);
  robot_goal_.GetPosition().coeffRef(0) = goal_point.getOrigin().getX();
  robot_goal_.GetPosition().coeffRef(1) = goal_point.getOrigin().getY();*/
  /*if (global_plan_.poses.size() - goal_idx < 5) {
    robot_goal_.GetTheta() = tf::getYaw(global_plan_.poses.back().pose.orientation);
    transformed_plan_.back().GetTheta() = robot_goal_.GetTheta();
  } else*/
  if (global_plan_overwrite_orientation_) {//为true  ***transformed_plan_.back()是全局坐标系下 局部规划目的地 ***  EstimateLocalGoalOrientation估计的朝向放入后面的optimal
    transformed_plan_.back().SetTheta(EstimateLocalGoalOrientation(transformed_plan_.back(), goal_idx));
  }
  if (transformed_plan_.size()==1) {// plan only contains the goal
    transformed_plan_.insert(transformed_plan_.begin(), robot_pose_); // insert start (not yet initialized)
  } else {
    transformed_plan_.front() = robot_pose_;// update start;
  }
  obst_vector_.clear();
  robot_goal_ = transformed_plan_.back();
//  while (obst_vector_.empty()) {
//    usleep(1);
    UpdateObstacleWithCostmap(robot_goal_.GetPosition());
//  }
  UpdateViaPointsContainer();
  bool micro_control = false;
  if (global_plan_.poses.back().pose.position.z == 1) {
    micro_control = true;
  }
  bool success = optimal_->Optimal(transformed_plan_, &robot_current_vel_, free_goal_vel_, micro_control);//transformed_plan_用于构建节点
  if (!success) {
    optimal_->ClearPlanner();
    roborts_common::ErrorInfo OptimalError(roborts_common::LP_OPTIMAL_ERROR, "optimal error");
    ROS_ERROR("optimal error");
    last_cmd_ = cmd_vel;
    return OptimalError;
  }
  bool feasible = optimal_->IsTrajectoryFeasible(teb_error_info_, robot_cost_.get(), robot_footprint_,
                                                 robot_inscribed_radius_, robot_circumscribed_radius, fesiable_step_look_ahead_);
  /*if (!feasible) {
    optimal_->ClearPlanner();
    last_cmd_ = cmd_vel;
    ROS_ERROR("trajectory is not feasible");
    roborts_common::ErrorInfo trajectory_error(roborts_common::LP_ALGORITHM_TRAJECTORY_ERROR, "trajectory is not feasible");
    return trajectory_error;
  }*/
  if (!optimal_->GetVelocity(teb_error_info_, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                             cmd_vel.accel.linear.x, cmd_vel.accel.linear.y, cmd_vel.accel.angular.z)) {
    optimal_->ClearPlanner();
    ROS_ERROR("can not get the velocity");
    roborts_common::ErrorInfo velocity_error(roborts_common::LP_VELOCITY_ERROR, "velocity is not feasible");
    last_cmd_ = cmd_vel;
    return velocity_error;
  }
  SaturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   max_vel_x_,max_vel_y_,max_vel_theta_,max_vel_x_backwards);
  last_cmd_ = cmd_vel;
  optimal_->Visualize();//显示局部规划的轨迹topic
  ROS_INFO("compute velocity succeed");
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

bool TebLocalPlanner::IsGoalReached () {

  tf::Stamped<tf::Pose> global_goal;
  tf::poseStampedMsgToTF(global_plan_.poses.back(), global_goal);
  global_goal.setData( plan_to_global_transform_ * global_goal );//plan_to_global_transform_通过UpdateGlobalToPlanTranform获得，为局部规划到全局之间的变换
  auto goal = DataConverter::LocalConvertTFData(global_goal);

  auto distance = (goal.first - robot_pose_.GetPosition()).norm();
  double delta_orient = g2o::normalize_theta( goal.second - robot_pose_.GetTheta());

  if (distance < xy_goal_tolerance_
      && fabs(delta_orient) < yaw_goal_tolerance_) {
    ROS_INFO("goal reached");
    return true;
  } else {
    return false;
  }

}
//在local_planner节点中先执行1.SetPlan  2.Initialize  3.ComputeVelocityCommand  4.IsGoalReached
bool TebLocalPlanner::SetPlan(const nav_msgs::Path& plan, const geometry_msgs::PoseStamped& goal) {
  if (plan_mutex_.try_lock()) {
    ROS_INFO("set plan");
    if (plan.poses.empty()) {
      temp_plan_.poses.push_back(goal);
    } else {
      temp_plan_ = plan;//全局坐标系下轨迹
    }
    plan_mutex_.unlock();
  }
}

bool TebLocalPlanner::GetPlan(const nav_msgs::Path& plan) {
  if (plan_mutex_.try_lock()) {
    global_plan_ = plan;
    plan_mutex_.unlock();
  }
}

bool TebLocalPlanner::PruneGlobalPlan() {
  if (global_plan_.poses.empty())  //global_plan_通过getPlan获得，局部规划坐标系下路径
    return true;
  try {
    tf::StampedTransform global_to_plan_transform;//全局转换到局部规划坐标系下的变换
    tf_.lock()->lookupTransform(global_plan_.poses.front().header.frame_id, robot_tf_pose_.frame_id_,ros::Time(0),global_to_plan_transform);//global_to_plan_transform是全局到规划frame之间的变换
    tf::Stamped<tf::Pose> robot;//局部规划坐标系下位姿
    robot.setData( global_to_plan_transform * robot_tf_pose_ );//robot_tf_pose_是机器人在全局坐标系下的位姿，robot是局部规划坐标系下的位姿
    //Eigen::Vector2d robot_to_goal(robot.getOrigin().x() - global_plan_.poses.back().pose.position.x,robot.getOrigin().y() - global_plan_.poses.back().pose.position.y);
    for (auto iterator = global_plan_.poses.begin(); iterator != global_plan_.poses.end(); ++iterator) {
      Eigen::Vector2d temp_vector (robot.getOrigin().x() - iterator->pose.position.x,
                                   robot.getOrigin().y() - iterator->pose.position.y);
      if (temp_vector.norm() < 0.8) {
        if (iterator == global_plan_.poses.begin()) {
          break;
        }
        global_plan_.poses.erase(global_plan_.poses.begin(), iterator);//擦除begin和iterator之间的数据，[ )
        break;
      }
    }
  }
  catch (const tf::TransformException& ex) {
   ROS_ERROR("prune global plan false, %s", ex.what());
    return false;
  }
  return true;
}
//在获得的i基础上，对plan_length<cut_lookahead_dist_内的全局位姿data_pose放入到  transformed_plan_  中，同时获得符合探测距离要求的i，具体为*current_goal_idx = i-1
bool TebLocalPlanner::TransformGlobalPlan(int *current_goal_idx) {
  transformed_plan_.clear();
  try {
    if (global_plan_.poses.empty()) {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }
    UpdateGlobalToPlanTranform();//由局部规划转换到全局坐标系
    tf::Stamped<tf::Pose> robot_pose;//当前机器人局部规划坐标系下位姿
    tf_.lock()->transformPose(global_plan_.poses.front().header.frame_id, robot_tf_pose_, robot_pose);//robot_tf_pose_为全局坐标系下位姿，robot_pose为局部规划frame下位姿
    double dist_threshold = std::max(costmap_->GetSizeXCell() * costmap_->GetResolution() / 2.0, costmap_->GetSizeYCell() * costmap_->GetResolution() / 2.0);//局部代价地图中心点x、y最大值
    dist_threshold *= 0.85;
    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;//局部避障可视的距离平方
    double sq_dist = 1e10;//机器人局部坐标系下位置距离global_plan_.pose[i]的距离平方
    double new_sq_dist = 0;
    while (i < (int)global_plan_.poses.size()) {
      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {
        sq_dist = new_sq_dist;
        break;
      }
      sq_dist = new_sq_dist;
      ++i;
    }
    tf::Stamped<tf::Pose> tf_pose;
    geometry_msgs::PoseStamped newer_pose;
    double plan_length = 0;
    //上面获取机器人后0.85米到终目的地的i，下面在获得的i基础上，对plan_length<cut_lookahead_dist_内的全局位姿data_pose放入到  transformed_plan_  中
    while(i < (int)global_plan_.poses.size() && sq_dist <= sq_dist_threshold && (cut_lookahead_dist_<=0 || plan_length <= cut_lookahead_dist_)) {
      const geometry_msgs::PoseStamped& pose = global_plan_.poses[i];//pose为局部规划坐标系下位姿
      tf::poseStampedMsgToTF(pose, tf_pose);//convert PoseStamped msg to Stamped
      //tf_pose.setData(plan_to_global_transform_ * tf_pose);//  设置数据元素，获得的tf_pose为  全局坐标系下位姿
      tf_pose.setData(tf_pose);
      // plan_to_global_transform_通过UpdateGlobalToPlanTranform获得，为局部规划到全局之间的变换
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;//全局坐标系
      tf::poseStampedTFToMsg(tf_pose, newer_pose);//convert Stamped  to PoseStamped msg
      auto temp = DataConverter::LocalConvertGData(newer_pose.pose);//Convert geometry_msgs::Pose to std::pair<Eigen::Vector2d, double>
      DataBase data_pose(temp.first, temp.second);//data_pose是DataBase类型 (const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
      transformed_plan_.push_back(data_pose);//global_plan_.pose[i]转化为全局坐标系下的via point
      double x_diff = robot_pose.getOrigin().x() - global_plan_.poses[i].pose.position.x;
      double y_diff = robot_pose.getOrigin().y() - global_plan_.poses[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (i>0 && cut_lookahead_dist_>0) {
        plan_length += Distance(global_plan_.poses[i-1].pose.position.x, global_plan_.poses[i-1].pose.position.y,
                                global_plan_.poses[i].pose.position.x, global_plan_.poses[i].pose.position.y);//计算局部规划路径的总长度，要保证小于cut_lookahead_dist_
      }
      ++i;
    }
    if (transformed_plan_.empty()) {
      tf::poseStampedMsgToTF(global_plan_.poses.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame_;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);
      auto temp = DataConverter::LocalConvertGData(newer_pose.pose);
      DataBase data_pose(temp.first, temp.second);// 如果transformed_plan_为空，则把局部规划的最后一个点转化为全局坐标系下的点data_pose
      transformed_plan_.push_back(data_pose);
      if (current_goal_idx) {
        *current_goal_idx = int(global_plan_.poses.size())-1;
      }
    } else {
      if (current_goal_idx) {
        *current_goal_idx = i-1;
      }
    }
  }
  catch(tf::LookupException& ex) {
    //LOG_ERROR << "transform error, " << ex.what();
    return false;
  }
  catch(tf::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s", ex.what());
    if (global_plan_.poses.size() > 0) {
      ROS_INFO("Global Frame: %s Plan Frame size %d : %s", global_frame_.c_str(), (unsigned int)global_plan_.poses.size(), global_plan_.poses[0].header.frame_id.c_str());
    }
    return false;
  }
  return true;
}
//moving_average_length有默认值=3   local_goal为全局坐标系下的局部规划目的地
double TebLocalPlanner::EstimateLocalGoalOrientation(const DataBase& local_goal, int current_goal_idx, int moving_average_length) const {
  int n = (int)global_plan_.poses.size();//global_plan_通过getPlan获得，局部规划坐标系下  所有轨迹、整体路径
  if (current_goal_idx > n-moving_average_length-2) {//检查是否靠近全局的目的地
    if (current_goal_idx >= n-1) {//已经到达目的地
      return local_goal.GetTheta(); //local_goal对应current_goal_idx，满足此条件表示local_goal为最终的目的地
    } else {
      tf::Quaternion global_orientation;//
      tf::quaternionMsgToTF(global_plan_.poses.back().pose.orientation, global_orientation);//convert Quaternion msg to Quaternion
      return  tf::getYaw(plan_to_global_transform_.getRotation() *  global_orientation );//返回全局坐标系下朝向，
      // plan_to_global_transform_通过UpdateGlobalToPlanTranform获得，为局部规划坐标系到全局坐标系之间的变换
    }
  }
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 );
  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k;//局部坐标系下位姿
  tf::Stamped<tf::Pose> tf_pose_kp1;//全局坐标系下位姿 下面为局部，对后面使用有影响
  const geometry_msgs::PoseStamped& pose_k = global_plan_.poses.at(current_goal_idx);//全局规划路径点序号current_goal_id对应的位姿（局部规划坐标系odom下），由GetPlan(temp_plan_)获得global_plan_；
  // ***transformed_plan_.back()是全局坐标系下 局部规划目的地 ***
  tf::poseStampedMsgToTF(pose_k, tf_pose_k);//convert PoseStamped msg to Stamped，tf_pose_k为局部规划坐标系下位姿
//  tf_pose_kp.setData(plan_to_global_transform_ * tf_pose_k);  //tf_pose_kp is local pose in local planner frame.tf_pose_kp1为全局坐标系下位姿
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {
    const geometry_msgs::PoseStamped& pose_k1 = global_plan_.poses.at(i+1);//pose_k1为局部规划坐标系odom下位姿
    tf::poseStampedMsgToTF(pose_k1, tf_pose_kp1);
    tf_pose_kp1.setData(plan_to_global_transform_ * tf_pose_kp1);//
    if(i==current_goal_idx)
        candidates.push_back(std::atan2(tf_pose_kp1.getOrigin().getY() - local_goal.GetPosition().y(), tf_pose_kp1.getOrigin().getX() - local_goal.GetPosition().x()));
    else
        candidates.push_back( std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                     tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX() ));
    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return AverageAngles(candidates);
}

void TebLocalPlanner::UpdateObstacleWithCostmap(Eigen::Vector2d local_goal) {

  //Eigen::Vector2d robot_orient = robot_pose_.OrientationUnitVec();
  Eigen::Vector2d goal_orient = local_goal - robot_pose_.GetPosition();

  for (unsigned int i=0; i<costmap_->GetSizeXCell()-1; ++i) {
    for (unsigned int j=0; j<costmap_->GetSizeYCell()-1; ++j) {
      if (costmap_->GetCost(i,j) == roborts_local_planner::LETHAL_OBSTACLE) {
        Eigen::Vector2d obs;
        costmap_->Map2World(i, j, obs.coeffRef(0), obs.coeffRef(1));

        Eigen::Vector2d obs_dir = obs - robot_pose_.GetPosition();
        if (obs_dir.dot(goal_orient) < 0
            && obs_dir.norm() > osbtacle_behind_robot_dist_) {
          continue;
        }

        obst_vector_.push_back(ObstaclePtr(new PointObstacle(obs)));
      }
    } // for y cell size
  }  // for x cell size

}

void TebLocalPlanner::UpdateViaPointsContainer() {

  via_points_.clear();

  double min_separation = param_config_.trajectory_opt().global_plan_viapoint_sep();
  if (min_separation<0) {
    return;
  }

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan_.size(); ++i) {// skip first one, since we do not need any point before the first min_separation [m]

    if (Distance( transformed_plan_[prev_idx].GetPosition().coeff(0), transformed_plan_[prev_idx].GetPosition().coeff(1),
                  transformed_plan_[i].GetPosition().coeff(0), transformed_plan_[i].GetPosition().coeff(1) ) < min_separation) {
      continue;
    }

    auto temp = transformed_plan_[i].GetPosition();
    via_points_.push_back(temp);
    prev_idx = i;
  }
}

void TebLocalPlanner::UpdateRobotPose() {
  local_cost_.lock()->GetRobotPose(robot_tf_pose_);//robot_tf_pose_机器人全局坐标系下位姿
  Eigen::Vector2d position;
  position.coeffRef(0) = robot_tf_pose_.getOrigin().getX();
  position.coeffRef(1) = robot_tf_pose_.getOrigin().getY();
  robot_pose_ = DataBase(position, tf::getYaw(robot_tf_pose_.getRotation()));//表示为二维位置和朝向角形式
}

void TebLocalPlanner::UpdateRobotVel() {
  tf::Stamped<tf::Pose> robot_vel_tf;
  odom_info_.GetVel(robot_vel_tf);
  robot_current_vel_.linear.x = robot_vel_tf.getOrigin().getX();
  robot_current_vel_.linear.y = robot_vel_tf.getOrigin().getY();
  robot_current_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
}
//UpdateGlobalToPlanTranform为局部规划到全局之间的变换求解
void TebLocalPlanner::UpdateGlobalToPlanTranform() {
  tf_.lock()->waitForTransform(global_frame_, ros::Time::now(),
                               global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                               global_plan_.poses.front().header.frame_id, ros::Duration(0.5));
  tf_.lock()->lookupTransform(global_frame_, ros::Time(),
                              global_plan_.poses.front().header.frame_id, global_plan_.poses.front().header.stamp,
                              global_plan_.poses.front().header.frame_id, plan_to_global_transform_);//global_plan_是局部规划frame下，plan_to_global_transform_是局部规划到全局坐标系之间的变换                           
}

void TebLocalPlanner::SaturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                      double max_vel_theta, double max_vel_x_backwards) const {
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  if (vx > max_vel_x) {
//    vx = max_vel_x;
      ratio_x = max_vel_x / vx;
  }

  if (max_vel_x_backwards<=0) {
    ROS_INFO("Do not choose max_vel_x_backwards to be <=0. "\
    "Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  } else if (vx < -max_vel_x_backwards) {
//    vx = -max_vel_x_backwards;
      ratio_x = -max_vel_x_backwards / vx;
  }
//  if (vy > max_vel_y) {
//    vy = max_vel_y;
//      ratio_y = max_vel_y / vy;
//  } else if (vy < -max_vel_y) {
//    vy = -max_vel_y;
//      ratio_y = -max_vel_y / vy;
//  }
  if (omega > max_vel_theta) {
//    omega = max_vel_theta;
      ratio_omega = max_vel_theta / omega;
  } else if (omega < -max_vel_theta) {
//    omega = -max_vel_theta;
      ratio_omega = -max_vel_theta / omega;
  }
  double ratio = std::min(ratio_x, ratio_omega);

  vx *= ratio;
//  vy *= ratio_y;
  omega *= ratio;
}

double TebLocalPlanner::ConvertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const {
  if (omega==0 || v==0) {
    return 0;
  }

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius) {
    radius = double(g2o::sign(radius)) * min_turning_radius;
  }

  return std::atan(wheelbase / radius);
}
//LocalVisualizationPtr visual在这里发布局部规划的轨迹
roborts_common::ErrorInfo TebLocalPlanner::Initialize (std::shared_ptr<roborts_costmap::CostmapInterface> local_cost,
                                  std::shared_ptr<tf::TransformListener> tf, LocalVisualizationPtr visual) {
  if (!is_initialized_) {
    oscillation_ = std::chrono::system_clock::now();
    tf_ = tf;
    local_cost_ = local_cost;
    std::string full_path = ros::package::getPath("roborts_planning") + "/local_planner/timed_elastic_band/config/timed_elastic_band.prototxt";
    roborts_common::ReadProtoFromTextFile(full_path.c_str(), &param_config_);
    if (&param_config_ == nullptr) {
      ROS_ERROR("error occur when loading config file");
      roborts_common::ErrorInfo read_file_error(roborts_common::ErrorCode::LP_ALGORITHM_INITILIZATION_ERROR, "load algorithm param file failed");
      is_initialized_ = false;
      ROS_ERROR("%s", read_file_error.error_msg().c_str());
      return read_file_error;
    }
    max_vel_x_          = param_config_.kinematics_opt().max_vel_x();
    max_vel_y_          = param_config_.kinematics_opt().max_vel_y();
    max_vel_theta_      = param_config_.kinematics_opt().max_vel_theta();
    max_vel_x_backwards = param_config_.kinematics_opt().max_vel_x_backwards();
    free_goal_vel_      = param_config_.tolerance_opt().free_goal_vel();
    xy_goal_tolerance_  = param_config_.tolerance_opt().xy_goal_tolerance();
    yaw_goal_tolerance_ = param_config_.tolerance_opt().yaw_goal_tolerance();
    cut_lookahead_dist_       = param_config_.trajectory_opt().max_global_plan_lookahead_dist();//数值为2
    fesiable_step_look_ahead_ = param_config_.trajectory_opt().feasibility_check_no_poses();
    osbtacle_behind_robot_dist_ = param_config_.obstacles_opt().costmap_obstacles_behind_robot_dist();
    global_plan_overwrite_orientation_ = param_config_.trajectory_opt().global_plan_overwrite_orientation();//true
    global_frame_ = local_cost_.lock()->GetGlobalFrameID();
    visual_ = visual;//将发布局部规划轨迹的topic对visual_赋值
    costmap_ = local_cost_.lock()->GetLayeredCostmap()->GetCostMap();
    robot_cost_ = std::make_shared<roborts_local_planner::RobotPositionCost>(*costmap_);
    obst_vector_.reserve(200);
    RobotFootprintModelPtr robot_model = GetRobotFootprintModel(param_config_);
    optimal_ = OptimalBasePtr(new TebOptimal(param_config_, &obst_vector_, robot_model, visual_, &via_points_));
    std::vector<geometry_msgs::Point> robot_footprint;
    robot_footprint = local_cost_.lock()->GetRobotFootprint();
    //robot_footprint_ = DataConverter::LocalConvertGData(robot_footprint);
    roborts_costmap::RobotPose robot_pose;
    local_cost_.lock()->GetRobotPose(robot_pose);
    auto temp_pose = DataConverter::LocalConvertRMData(robot_pose);
    robot_pose_ = DataBase(temp_pose.first, temp_pose.second);
    robot_footprint_.push_back(robot_pose_.GetPosition());
    last_robot_pose_ = robot_pose_;
    RobotPositionCost::CalculateMinAndMaxDistances(robot_footprint_,robot_inscribed_radius_,robot_circumscribed_radius);
    odom_info_.SetTopic(param_config_.opt_frame().odom_frame());//为odom
    is_initialized_ = true;
    ROS_INFO("local algorithm initialize ok");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
  }
  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

void TebLocalPlanner::RegisterErrorCallBack(ErrorInfoCallback error_callback) {
  error_callback_ = error_callback;
}

bool TebLocalPlanner::CutAndTransformGlobalPlan(int *current_goal_idx) {
  if (!transformed_plan_.empty()) {
    transformed_plan_.clear();
  }

}

bool TebLocalPlanner::SetPlanOrientation() {
  if (global_plan_.poses.size() < 2) {
    ROS_WARN("can not compute the orientation because the global plan size is: %d", (int)global_plan_.poses.size());
    return false;
  } else {
    //auto goal = DataConverter::LocalConvertGData(global_plan_.poses.back().pose);
    //auto line_vector = (robot_pose_.GetPosition() - goal.first);
    //auto  orientation = GetOrientation(line_vector);
    for (int i = 0; i < global_plan_.poses.size() - 1; ++i) {
      auto pose = DataConverter::LocalConvertGData(global_plan_.poses[i].pose);
      auto next_pose = DataConverter::LocalConvertGData(global_plan_.poses[i+1].pose);
      double x = global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x;
      double y = global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y;
      double angle = atan2(y, x);
      auto quaternion = EulerToQuaternion(0, 0, angle);
      global_plan_.poses[i].pose.orientation.w = quaternion[0];
      global_plan_.poses[i].pose.orientation.x = quaternion[1];
      global_plan_.poses[i].pose.orientation.y = quaternion[2];
      global_plan_.poses[i].pose.orientation.z = quaternion[3];
    }
  }
}

} // namespace roborts_local_planner
