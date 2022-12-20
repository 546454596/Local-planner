#include "timed_elastic_band/teb_vertex_console.h"

namespace roborts_local_planner {

TebVertexConsole::TebVertexConsole() { }

TebVertexConsole::~TebVertexConsole() {
  ClearAllVertex();
}

void TebVertexConsole::AddPose(const DataBase& pose, bool fixed) {
  TebVertexPose *pose_vertex = new TebVertexPose(pose, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TebVertexConsole::AddPose(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, bool fixed){
  auto a = DataBase(position, theta);
  TebVertexPose *pose_vertex = new TebVertexPose(a, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TebVertexConsole::AddTimeDiff(double dt, bool fixed) {
  TebVertexTimeDiff *timediff_vertex = new TebVertexTimeDiff(dt, fixed);
  timediff_vec_.push_back(timediff_vertex);
  return;
}

void TebVertexConsole::AddPoseAndTimeDiff(const DataBase &pose, double dt){
  if (SizePoses() != SizeTimeDiffs()) {
    AddPose(pose, false);
    AddTimeDiff(dt, false);
  } else {
  }
  return;
}

void TebVertexConsole::AddPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d> &position, double theta, double dt){
  if (SizePoses() != SizeTimeDiffs()) {
    AddPose(position, theta, false);
    AddTimeDiff(dt, false);
  } else {
  }
  return;
}

void TebVertexConsole::DeletePose(int index){
  delete pose_vec_.at(index);
  pose_vec_.erase(pose_vec_.begin() + index);
}

void TebVertexConsole::DeletePose(int index, int number) {
  for (int i = index; i < index + number; ++i) {
    delete pose_vec_.at(i);
  }
  pose_vec_.erase(pose_vec_.begin() + index, pose_vec_.begin() + index + number);
}

void TebVertexConsole::DeleteTimeDiff(int index) {
  delete timediff_vec_.at(index);
  timediff_vec_.erase(timediff_vec_.begin() + index);
}

void TebVertexConsole::DeleteTimeDiff(int index, int number) {
  for (int i = index; i < index + number; ++i) {
    delete timediff_vec_.at(i);
  }
  timediff_vec_.erase(timediff_vec_.begin() + index, timediff_vec_.begin() + index + number);
}

void TebVertexConsole::InsertPose(int index, const DataBase &pose) {
  TebVertexPose *pose_vertex = new TebVertexPose(pose);
  pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
}

void TebVertexConsole::InsertPose(int index, const Eigen::Ref<const Eigen::Vector2d> &position, double theta) {
  auto temp_data = DataBase(position, theta);
  TebVertexPose *pose_vertex = new TebVertexPose(temp_data);
  pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
}

void TebVertexConsole::InsertTimeDiff(int index, double dt) {
  TebVertexTimeDiff *timediff_vertex = new TebVertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin() + index, timediff_vertex);
}

void TebVertexConsole::ClearAllVertex(){
  for (PoseSequence::iterator pose_it = pose_vec_.begin(); pose_it != pose_vec_.end(); ++pose_it) {
    delete *pose_it;
  }
  pose_vec_.clear();

  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it) {
    delete *dt_it;
  }
  timediff_vec_.clear();
}

void TebVertexConsole::SetPoseVertexFixed(int index, bool status) {
  pose_vec_.at(index)->setFixed(status);
}

void TebVertexConsole::SetTimeDiffVertexFixed(int index, bool status) {
  timediff_vec_.at(index)->setFixed(status);
}

void TebVertexConsole::AutoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples) {
  bool modified = true;
  for (int rep = 0; rep < 100 && modified; ++rep) {
    modified = false;
    for (int i = 0; i < SizeTimeDiffs(); ++i) {
      if (TimeDiff(i) > dt_ref + dt_hysteresis && SizeTimeDiffs() < max_samples) {
        double newtime = 0.5 * TimeDiff(i);
        TimeDiff(i) = newtime;
        DataBase temp_pose;
        temp_pose.AverageInPlace(Pose(i), Pose(i + 1));
        InsertPose(i + 1, temp_pose);
        InsertTimeDiff(i + 1, newtime);
        modified = true;
      } else if (TimeDiff(i) < dt_ref - dt_hysteresis
          && SizePoses() > min_samples) {
        if (i < ((int) SizeTimeDiffs() - 1)) {
          TimeDiff(i + 1) = TimeDiff(i + 1) + TimeDiff(i);
          DeleteTimeDiff(i);
          DeletePose(i + 1);
        }
        modified = true;
      }
    }
  }
}

double TebVertexConsole::GetSumOfAllTimeDiffs() const {
  double time = 0;

  for (TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it) {
    time += (*dt_it)->GetDiffTime();
  }
  return time;
}

double TebVertexConsole::GetAccumulatedDistance() const {
  double dist = 0;

  for (int i = 1; i < SizePoses(); ++i) {
    dist += (Pose(i).GetPosition() - Pose(i - 1).GetPosition()).norm();
  }
  return dist;
}

bool TebVertexConsole::InitTEBtoGoal(const DataBase &start,
                                     const DataBase &goal,
                                     double diststep,
                                     double max_vel_x,
                                     int min_samples,
                                     bool guess_backwards_motion) {
  if (!IsInit()) {
    AddPose(start);
    SetPoseVertexFixed(0, true);

    double timestep = 0.1;
    if (diststep != 0) {
      Eigen::Vector2d point_to_goal = goal.GetPosition() - start.GetPosition();
      double dir_to_goal = std::atan2(point_to_goal.coeffRef(1), point_to_goal.coeffRef(0));
      double dx = diststep * std::cos(dir_to_goal);
      double dy = diststep * std::sin(dir_to_goal);
      double orient_init = dir_to_goal;

      if (guess_backwards_motion && point_to_goal.dot(start.OrientationUnitVec()) < 0) {
        //orient_init = g2o::normalize_theta(orient_init + M_PI);
      }

      double dist_to_goal = point_to_goal.norm();
      double no_steps_d = dist_to_goal / std::abs(diststep);
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d);

      if (max_vel_x > 0) {
        timestep = diststep / max_vel_x;
      }

      for (unsigned int i = 1; i <= no_steps; i++) {
        if (i == no_steps && no_steps_d == (float) no_steps) {
          break;
        }

        auto temp_data = DataConverter::LocalConvertCData(start.GetPosition().coeffRef(0) + i*dx,
                                                          start.GetPosition().coeffRef(1) + i*dy,
                                                          orient_init);
        AddPoseAndTimeDiff(temp_data.first, temp_data.second, timestep);
      }
    }
    if (SizePoses() < min_samples - 1) {
      while (SizePoses() < min_samples - 1) {
        DataBase temp_pose;
        temp_pose.AverageInPlace(BackPose(), goal);
        if (max_vel_x > 0) {
          timestep = (temp_pose.GetPosition() - goal.GetPosition()).norm() / max_vel_x;
        }
        AddPoseAndTimeDiff(temp_pose, timestep);
      }
    }
    if (max_vel_x > 0) {
      timestep = (goal.GetPosition() - BackPose().GetPosition()).norm() / max_vel_x;
    }
    AddPoseAndTimeDiff(goal, timestep);
    SetPoseVertexFixed(SizePoses() - 1, true);
  } else  {
    return false;
  }
  return true;
}
//estimate_orient为true  min_samples=3     plan是transformed_plan_，是全局坐标系下 局部规划轨迹
bool TebVertexConsole::InitTEBtoGoal(std::vector<DataBase> &plan, double dt, bool estimate_orient, int min_samples,
                                     bool guess_backwards_motion, bool micro_control) {
  if (!IsInit()) {
    DataBase start = plan.front();//全局坐标系下 局部规划起点
    DataBase goal = plan.back();//全局坐标系下 局部规划目的地
    AddPose(start);
    SetPoseVertexFixed(0, true);//index为0的节点设置为fixed
    bool backwards = false;//起点和局部目的地之间夹角为钝角时，则可以后退行驶。
    /*if (guess_backwards_motion
        && (goal.GetPosition() - start.GetPosition()).dot(start.OrientationUnitVec()) < 0) {
      backwards = true;
    }*/
    for (int i = 1; i < (int) plan.size() - 1; ++i) {
      double yaw;
      if (estimate_orient && !micro_control) {//此处都为true
        double dx = plan[i + 1].GetPosition().coeffRef(0) - plan[i].GetPosition().coeffRef(0);
        double dy = plan[i + 1].GetPosition().coeffRef(1) - plan[i].GetPosition().coeffRef(1);
        /*if (backwards) {
          yaw = g2o::normalize_theta(std::atan2(dy, dx) + M_PI);
          plan[i].SetTheta(yaw);
        } else*/ {
          plan[i].SetTheta(std::atan2(dy, dx));
        }
      }
      AddPoseAndTimeDiff(plan[i], dt);//dt=0.5  此处添加的位姿和dt都不是固定的
    }
    if (SizePoses() < min_samples - 1) {
      while (SizePoses() < min_samples - 1) {
        DataBase temp_pose;
        temp_pose.AverageInPlace(BackPose(), goal);
        AddPoseAndTimeDiff(temp_pose, dt);
      }
    }
    AddPoseAndTimeDiff(goal, dt);
    SetPoseVertexFixed(SizePoses() - 1, true);
  } else {
    return false;
  }
  return true;
}

int TebVertexConsole::FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_point,
                                                double *distance,
                                                int begin_idx) const {
  std::vector<double> dist_vec;
  dist_vec.reserve(SizePoses());

  int n = SizePoses();

  for (int i = begin_idx; i < n; i++) {
    Eigen::Vector2d diff = ref_point - Pose(i).GetPosition();
    dist_vec.push_back(diff.norm());
  }

  if (dist_vec.empty())
    return -1;

  int index_min = 0;

  double last_value = dist_vec.at(0);
  for (int i = 1; i < (int) dist_vec.size(); i++) {
    if (dist_vec.at(i) < last_value) {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance) {
    *distance = last_value;
  }
  return begin_idx + index_min;
}

int TebVertexConsole::FindClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d> &ref_line_start,
                                                const Eigen::Ref<const Eigen::Vector2d> &ref_line_end,
                                                double *distance) const {
  std::vector<double> dist_vec;
  dist_vec.reserve(SizePoses());

  int n = SizePoses();


  for (int i = 0; i < n; i++) {
    Eigen::Vector2d point = Pose(i).GetPosition();
    double diff = DistancePointToSegment2D(point, ref_line_start, ref_line_end);
    dist_vec.push_back(diff);
  }

  if (dist_vec.empty()) {
    return -1;
  }

  int index_min = 0;

  double last_value = dist_vec.at(0);
  for (int i = 1; i < (int) dist_vec.size(); i++) {
    if (dist_vec.at(i) < last_value) {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance) {
    *distance = last_value;
  }

  return index_min;
}

int TebVertexConsole::FindClosestTrajectoryPose(const Point2dContainer &vertices, double *distance) const {
  if (vertices.empty()) {
    return 0;
  } else if (vertices.size() == 1) {
    return FindClosestTrajectoryPose(vertices.front());
  } else if (vertices.size() == 2) {
    return FindClosestTrajectoryPose(vertices.front(), vertices.back());
  }

  std::vector<double> dist_vec;
  dist_vec.reserve(SizePoses());

  int n = SizePoses();

  for (int i = 0; i < n; i++) {
    Eigen::Vector2d point = Pose(i).GetPosition();
    double diff = HUGE_VAL;
    for (int j = 0; j < (int) vertices.size() - 1; ++j) {
      diff = std::min(diff, DistancePointToSegment2D(point, vertices[j], vertices[j + 1]));
    }
    diff = std::min(diff, DistancePointToSegment2D(point, vertices.back(), vertices.front()));
    dist_vec.push_back(diff);
  }

  if (dist_vec.empty()) {
    return -1;
  }

  int index_min = 0;

  double last_value = dist_vec.at(0);
  for (int i = 1; i < (int) dist_vec.size(); i++) {
    if (dist_vec.at(i) < last_value) {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance) {
    *distance = last_value;
  }

  return index_min;
}

int TebVertexConsole::FindClosestTrajectoryPose(const Obstacle &obstacle, double *distance) const {
  const PointObstacle *pobst = dynamic_cast<const PointObstacle *>(&obstacle);
  if (pobst) {
    return FindClosestTrajectoryPose(pobst->Position(), distance);
  }

  const LineObstacle *lobst = dynamic_cast<const LineObstacle *>(&obstacle);
  if (lobst) {
    return FindClosestTrajectoryPose(lobst->Start(), lobst->End(), distance);
  }

  const PolygonObstacle *polyobst = dynamic_cast<const PolygonObstacle *>(&obstacle);
  if (polyobst) {
    return FindClosestTrajectoryPose(polyobst->Vertices(), distance);
  }

  return FindClosestTrajectoryPose(obstacle.GetCentroid(), distance);
}

bool TebVertexConsole::DetectDetoursBackwards(double threshold) const {
  if (SizePoses() < 2) {
    return false;
  }

  Eigen::Vector2d d_start_goal = BackPose().GetPosition() - Pose(0).GetPosition();
  d_start_goal.normalize();

  for (int i = 0; i < SizePoses(); ++i) {
    Eigen::Vector2d orient_vector(cos(Pose(i).GetTheta()), sin(Pose(i).GetTheta()));
    if (orient_vector.dot(d_start_goal) < threshold) {

      return true;
    }
  }

  return false;
}

void TebVertexConsole::UpdateAndPruneTEB(boost::optional<const DataBase &> new_start, boost::optional<const DataBase &> new_goal,
                                         int min_samples){
  if (new_start && SizePoses() > 0) {
    double dist_cache = (new_start->GetPosition() - Pose(0).GetPosition()).norm();
    double dist;
    int lookahead = std::min<int>(SizePoses() - min_samples, 10);
    int nearest_idx = 0;
    for (int i = 1; i <= lookahead; ++i) {
      dist = (new_start->GetPosition() - Pose(i).GetPosition()).norm();
      if (dist < dist_cache) {
        dist_cache = dist;
        nearest_idx = i;
      } else {
        break;
      }
    }

    if (nearest_idx > 0) {
      DeletePose(1, nearest_idx);
      DeleteTimeDiff(1, nearest_idx);
    }

    Pose(0) = *new_start;
  }

  if (new_goal && SizePoses() > 0) {
    BackPose() = *new_goal;
  }
}

bool TebVertexConsole::IsTrajectoryInsideRegion(double radius, double max_dist_behind_robot, int skip_poses) {
  if (SizePoses() <= 0){
    return true;
  }

  double radius_sq = radius * radius;
  double max_dist_behind_robot_sq = max_dist_behind_robot * max_dist_behind_robot;
  Eigen::Vector2d robot_orient = Pose(0).OrientationUnitVec();

  for (int i = 1; i < SizePoses(); i = i + skip_poses + 1) {
    Eigen::Vector2d dist_vec = Pose(i).GetPosition() - Pose(0).GetPosition();
    double dist_sq = dist_vec.squaredNorm();

    if (dist_sq > radius_sq) {
      return false;
    }

    if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 && dist_sq > max_dist_behind_robot_sq) {
      return false;
    }

  }
  return true;
}

} // namespace roborts_local_planner