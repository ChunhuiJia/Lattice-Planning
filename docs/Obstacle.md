# Class Obstacle

说明：这个类由于比较复杂，也比较重要，所以专门抽出来学习分析

概况：这是一个将Obstacle与其路径属性关联起来的类。障碍物相对于路径的路径属性。s和l是路径属性的例子。

障碍物的决策也和路径相关，决策分为两类：横向决策和纵向决策。

横向决策包括：nudge(轻推，可理解为向一侧稍微挪动)和ignore(忽视)，且优先级是nudge>ignore。

纵向决策包括：stop>yield(让行)>=follow>overtake(超车)>ignore

ignore决策优先级最低。

代码路径：/modules/planning/common/obstacle.h

# 首先看看有哪些public的成员函数

## 1.构造函数

### Obstacle()

```cpp
Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()) {
  is_caution_level_obstacle_ = (obstacle_priority == ObstaclePriority::CAUTION);
  std::vector<common::math::Vec2d> polygon_points;
  if (FLAGS_use_navigation_mode ||
      perception_obstacle.polygon_point_size() <= 2) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    ACHECK(perception_obstacle.polygon_point_size() > 2)
        << "object " << id << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }
  ACHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull.\n"
      << perception_obstacle.DebugString();

  is_static_ = (is_static || obstacle_priority == ObstaclePriority::IGNORE);
  is_virtual_ = (perception_obstacle.id() < 0);
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());
}
```

### Obstacle()

```cpp
Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const bool is_static)
    : Obstacle(id, perception_obstacle, obstacle_priority, is_static) {
  trajectory_ = trajectory;
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (trajectory_points.size() > 0) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s +=
        common::util::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}
```

## 2.public计算类函数

### GetPointAtTime() 

```cpp
common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}
```

### GetBoundingBox()

```cpp
common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.length(),
                             perception_obstacle_.width());
}
```



### CreateObstacles()

brief:这是一个辅助函数，可以从预测数据中创建障碍物，对于每个障碍物，原始预测可能有多个轨迹，这个函数会为每个轨迹创建一个障碍物。

```cpp
std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  for (const auto& prediction_obstacle : predictions.prediction_obstacle()) {
    if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle())) {
      AERROR << "Invalid perception obstacle: "
             << prediction_obstacle.perception_obstacle().DebugString();
      continue;
    }
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    if (prediction_obstacle.trajectory().empty()) {
      obstacles.emplace_back(
          new Obstacle(perception_id, prediction_obstacle.perception_obstacle(),
                       prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      continue;
    }

    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      bool is_valid_trajectory = true;
      for (const auto& point : trajectory.trajectory_point()) {
        if (!IsValidTrajectoryPoint(point)) {
          AERROR << "obj:" << perception_id
                 << " TrajectoryPoint: " << trajectory.ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        continue;
      }

      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      obstacles.emplace_back(
          new Obstacle(obstacle_id, prediction_obstacle.perception_obstacle(),
                       trajectory, prediction_obstacle.priority().priority(),
                       prediction_obstacle.is_static()));
      ++trajectory_index;
    }
  }
  return obstacles;
}
```

### CreateStaticVirtualObstacles()

```cpp
std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}
```

### IsValidPerceptionObstacle()

```cpp
bool Obstacle::IsValidPerceptionObstacle(const PerceptionObstacle& obstacle) {
  if (obstacle.length() <= 0.0) {
    AERROR << "invalid obstacle length:" << obstacle.length();
    return false;
  }
  if (obstacle.width() <= 0.0) {
    AERROR << "invalid obstacle width:" << obstacle.width();
    return false;
  }
  if (obstacle.height() <= 0.0) {
    AERROR << "invalid obstacle height:" << obstacle.height();
    return false;
  }
  if (obstacle.has_velocity()) {
    if (std::isnan(obstacle.velocity().x()) ||
        std::isnan(obstacle.velocity().y())) {
      AERROR << "invalid obstacle velocity:"
             << obstacle.velocity().DebugString();
      return false;
    }
  }
  for (auto pt : obstacle.polygon_point()) {
    if (std::isnan(pt.x()) || std::isnan(pt.y())) {
      AERROR << "invalid obstacle polygon point:" << pt.DebugString();
      return false;
    }
  }
  return true;
}
```

### IsValidTrajectoryPoint()

```cpp
bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}
```

### IsValidObstacle()

```cpp
bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}
```

### MinRadiusStopDistance()

brief:使用ADC的最小转弯半径计算与障碍物的停止距离

```cpp
double Obstacle::MinRadiusStopDistance(
    const common::VehicleParam& vehicle_param) const {
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  static constexpr double stop_distance_buffer = 0.5;
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  double lateral_diff =
      vehicle_param.width() / 2.0 + std::max(std::fabs(sl_boundary_.start_l()),
                                             std::fabs(sl_boundary_.end_l()));
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  double stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;
  stop_distance -= vehicle_param.front_edge_to_center();
  stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
  stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
  return stop_distance;
}
```

### BuildReferenceLineStBoundary()

```cpp
void Obstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                            const double adc_start_s) {
  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double adc_width = adc_param.width();
  if (is_static_ || trajectory_.trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s();
    double end_s = sl_boundary_.end_s();
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }
    if (!reference_line.IsBlockRoad(perception_bounding_box_, adc_width)) {
      return;
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = STBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}
```

### CheckLaneBlocking()

```cpp
void Obstacle::CheckLaneBlocking(const ReferenceLine& reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }
  DCHECK(sl_boundary_.has_start_s());
  DCHECK(sl_boundary_.has_end_s());
  DCHECK(sl_boundary_.has_start_l());
  DCHECK(sl_boundary_.has_end_l());

  if (sl_boundary_.start_l() * sl_boundary_.end_l() < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  const double driving_width = reference_line.GetDrivingWidth(sl_boundary_);
  auto vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

  if (reference_line.IsOnLane(sl_boundary_) &&
      driving_width <
          vehicle_param.width() + FLAGS_static_obstacle_nudge_l_buffer) {
    is_lane_blocking_ = true;
    return;
  }

  is_lane_blocking_ = false;
}
```



## 3.public set函数

### SetId()

```cpp
void SetId(const std::string& id) { id_ = id; }
```

### AddLongitudinalDecision()

```cpp
void Obstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}
```

### AddLateralDecision()

```cpp
void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}
```

### set_path_st_boundary()

```cpp
void Obstacle::set_path_st_boundary(const STBoundary& boundary) {
  path_st_boundary_ = boundary;
  path_st_boundary_initialized_ = true;
}
```

### SetStBoundaryType()

```cpp
void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType type) {
  path_st_boundary_.SetBoundaryType(type);
}
```

### EraseStBoundary()

```cpp
void Obstacle::EraseStBoundary() { path_st_boundary_ = STBoundary(); }
```

### SetReferenceLineStBoundary()

```cpp
void Obstacle::SetReferenceLineStBoundary(const STBoundary& boundary) {
  reference_line_st_boundary_ = boundary;
}
```

### SetReferenceLineStBoundaryType()

```cpp
void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}
```

### EraseReferenceLineStBoundary()

```cpp
void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = STBoundary();
}
```

### SetPerceptionSlBoundary()

```cpp
void Obstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
  sl_boundary_ = sl_boundary;
}
```

### SetBlockingObstacle()

```cpp
void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
```

### SetLaneChangeBlocking()

```cpp
void SetLaneChangeBlocking(const bool is_distance_clear);
```



## 4.public查询函数

### Id()

```cpp
  const std::string& Id() const { return id_; }
```

### speed()

```cpp
double speed() const { return speed_; }
```

### PerceptionId()

```cpp
  int32_t PerceptionId() const { return perception_id_; }
```

### IsStatic()

```cpp
bool IsStatic() const { return is_static_; }
```

### IsVirtual() 

```cpp
bool IsVirtual() const { return is_virtual_; }
```

### PerceptionBoundingBox() 

```cpp
const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
```

### PerceptionPolygon() 

```c++
const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
```

### Trajectory()

```cpp
const prediction::Trajectory& Trajectory() const { return trajectory_; }
```

### AddTrajectoryPoint()

```cpp
common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
```

### bool HasTrajectory()

```cpp
bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }
```

### Perception()

```cpp
const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }
```

### IsCautionLevelObstacle()

```cpp
inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }
```

### LateralDecision()

brief:返回横向的决策，one of {nudge,ignore}

```cpp
const ObjectDecisionType& Obstacle::LateralDecision() const {
  return lateral_decision_;
}
```

### LongitudinalDecision()

```cpp
const ObjectDecisionType& Obstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}
```

### DebugString()

```cpp
std::string Obstacle::DebugString() const {
  std::stringstream ss;
  ss << "Obstacle id: " << id_;
  for (size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitudinal decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}
```

### PerceptionSLBoundary()

```cpp
const SLBoundary& Obstacle::PerceptionSLBoundary() const {
  return sl_boundary_;
}
```

### reference_line_st_boundary

```cpp
const STBoundary& Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}
```

### path_st_boundary()

```cpp
const STBoundary& Obstacle::path_st_boundary() const {
  return path_st_boundary_;
}
```

### decider_tags()

```cpp
const std::vector<std::string>& Obstacle::decider_tags() const {
  return decider_tags_;
}
```

### decisions()

```cpp
const std::vector<ObjectDecisionType>& Obstacle::decisions() const {
  return decisions_;
}
```

### HasLateralDecision()

```cpp
bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}
```

### is_path_st_boundary_initialized()

```cpp
bool is_path_st_boundary_initialized() {
    return path_st_boundary_initialized_;
  }
```

### HasLongitudinalDecision()

```cpp
bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}
```

### HasNonIgnoreDecision()

```cpp
bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}
```

### IsIgnore()

brief:检查该对象是否可以安全地忽略，如果横向决策忽略且纵向决策忽略，则该对象将被忽略，返回longitudinal_decision_ == ignore && lateral_decision == ignore

```cpp
bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}
```

### IsLongitudinalIgnore()

```cpp
bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}
```

### IsLateralIgnore()

```cpp
bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}
```

### IsLongitudinalDecision()

brief:检查ObjectDecisionType是否为纵向决策。

```cpp
bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}
```

### IsLateralDecision()

```cpp
bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}
```

### IsBlockingObstacle()

```cpp
bool IsBlockingObstacle() const { return is_blocking_obstacle_; }
```

### IsLaneBlocking()

```cpp
bool IsLaneBlocking() const { return is_lane_blocking_; }
```

### IsLaneChangeBlocking()

```cpp
bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
```

## 5.private成员函数

### MergeLongitudinalDecision()

```cpp
ObjectDecisionType Obstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  // stop compiler complaining
}
```

### MergeLateralDecision()

```cpp
ObjectDecisionType Obstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      DCHECK(lhs.nudge().type() == rhs.nudge().type())
          << "could not merge left nudge and right nudge";
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
  return lhs;
}
```

### BuildTrajectoryStBoundary()

```cpp
bool Obstacle::BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                         const double adc_start_s,
                                         STBoundary* const st_boundary) {
  if (!IsValidObstacle(perception_obstacle_)) {
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception_obstacle_.DebugString();
    return false;
  }
  const double object_width = perception_obstacle_.width();
  const double object_length = perception_obstacle_.length();
  const auto& trajectory_points = trajectory_.trajectory_point();
  if (trajectory_points.empty()) {
    AWARN << "object " << id_ << " has no trajectory points";
    return false;
  }
  const auto& adc_param =
      VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double adc_length = adc_param.length();
  const double adc_half_length = adc_length / 2.0;
  const double adc_width = adc_param.width();
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  std::vector<std::pair<STPoint, STPoint>> polygon_points;

  SLBoundary last_sl_boundary;
  int last_index = 0;

  for (int i = 1; i < trajectory_points.size(); ++i) {
    ADEBUG << "last_sl_boundary: " << last_sl_boundary.ShortDebugString();

    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double object_moving_box_length =
        object_length + common::util::DistanceXY(first_point, second_point);

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(
        center, first_point.theta(), object_moving_box_length, object_width);
    SLBoundary object_boundary;
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    // roughly skip points that are too close to last_sl_boundary box
    const double distance_xy =
        common::util::DistanceXY(trajectory_points[last_index].path_point(),
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;
    }

    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) / 2.0;
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    const double end_s = (i == 1) ? reference_line.Length()
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);

    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }

    // update history record
    last_sl_boundary = object_boundary;
    last_index = i;

    // skip if object is entirely on one side of reference line.
    static constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (!IsCautionLevelObstacle() &&
        (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
             skip_l_distance ||
         std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
             -skip_l_distance)) {
      continue;
    }

    if (!IsCautionLevelObstacle() && object_boundary.end_s() < 0) {
      // skip if behind reference line
      continue;
    }
    static constexpr double kSparseMappingS = 20.0;
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS
            : kStBoundaryDeltaS;
    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s =
        std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap(
            {low_ref, low_ref.heading(), adc_length,
             adc_width + FLAGS_nonstatic_obstacle_nudge_l_buffer});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap(
            {high_ref, high_ref.heading(), adc_length,
             adc_width + FLAGS_nonstatic_obstacle_nudge_l_buffer});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t =
          (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      polygon_points.emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }
  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = STBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}
```

### IsValidObstacle()

```cpp
bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}
```

## 6.private成员变量

  std::string id_;

  int32_t perception_id_ = 0;

  bool is_static_ = false;

  bool is_virtual_ = false;

  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  prediction::Trajectory trajectory_;

  perception::PerceptionObstacle perception_obstacle_;

  common::math::Box2d perception_bounding_box_;

  common::math::Polygon2d perception_polygon_;

  std::vector<ObjectDecisionType> decisions_;

  std::vector<<std::string>> decider_tags_;

  SLBoundary sl_boundary_;

  STBoundary reference_line_st_boundary_;

  STBoundary path_st_boundary_;

  ObjectDecisionType lateral_decision_;

  ObjectDecisionType longitudinal_decision_;

  bool is_blocking_obstacle_ = false;   //仅用于keep_clear

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

struct ObjectTagCaseHash

```
struct ObjectTagCaseHash {
    size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  }
```

s_lateral_decision_safety_sorter_

```
static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,

​                                  ObjectTagCaseHash>

​      s_lateral_decision_safety_sorter_;
```

s_longitudinal_decision_safety_sorter_

```
static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
}
```

typedef IndexedList<std::string, Obstacle> IndexedObstacles;

typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

