# Class ReferenceLineInfo

说明：这个类由于比较复杂，也比较重要，所以专门抽出来学习分析

概况：

> 建立统一的数据结构：ReferenceLineInfo和Obstacle。
>
> ReferenceLineInfo是通过把所有的上游信息投影到ReferenceLine上，为规划提供依据，
>
> Obstacle是想把所有的上游信息抽象成一个叫Obstacle的抽象障碍物
>
> 参考文献：[Apollo Planning代码学习一，后来博主再也没更新过了](https://www.zhihu.com/column/c_1194672142837530624)

代码路径：/modules/planning/common/reference_line_info.h和.cpp

# 首先看看有哪些成员public的函数：

## 1.构造或初始化函数

### ReferenceLineInfo()

### Init()

## 2.public计算类函数

### AddObstacles()

### AddObstacle()

### SDistanceToDestination()

```cpp
double ReferenceLineInfo::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return stop_s - adc_sl_boundary_.end_s();
}
```

### ReachedDestination()

```cpp
bool ReferenceLineInfo::ReachedDestination() const {
  static constexpr double kDestinationDeltaS = 0.05;
  const double distance_destination = SDistanceToDestination();
  return distance_destination <= kDestinationDeltaS;
}
```

### AddCost()

```cpp
void AddCost(double cost) { cost_ += cost; }
```

### LocateLaneInfo()

```cpp
hdmap::LaneInfoConstPtr ReferenceLineInfo::LocateLaneInfo(
    const double s) const {
  std::vector<hdmap::LaneInfoConstPtr> lanes;
  reference_line_.GetLaneFromS(s, &lanes);
  if (lanes.empty()) {
    AWARN << "cannot get any lane using s";
    return nullptr;
  }

  return lanes.front();
}
```

### GetNeighborLaneInfo()

```cpp
bool ReferenceLineInfo::GetNeighborLaneInfo(
    const ReferenceLineInfo::LaneType lane_type, const double s,
    hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
  auto ptr_lane_info = LocateLaneInfo(s);
  if (ptr_lane_info == nullptr) {
    return false;
  }

  switch (lane_type) {
    case LaneType::LeftForward: {
      if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::LeftReverse: {
      if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
      break;
    }
    case LaneType::RightForward: {
      if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
      break;
    }
    case LaneType::RightReverse: {
      if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
        return false;
      }
      *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
      break;
    }
    default:
      ACHECK(false);
  }
  auto ptr_neighbor_lane =
      hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*ptr_lane_id);
  if (ptr_neighbor_lane == nullptr) {
    return false;
  }

  auto ref_point = reference_line_.GetReferencePoint(s);

  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
                                        &neighbor_s, &neighbor_l)) {
    return false;
  }

  *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
  return true;
}
```

### IsStartFrom() 

brief:检查当前的参考线是否是从另一个参考线信息线开始，方法是检查当前参考线的起点是否在上一条参考线信息上。

return ：如果当前参考线从上一条参考线开始，则返回true，否则返回false.

```cpp
bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}
```

### CombinePathAndSpeedProfile()

brief:通过某种配置将最终结果聚合在一起

```cpp
bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;

  if (path_data_.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point =
        path_data_.GetPathPointWithPathS(speed_point.s());
    path_point.set_s(path_point.s() + start_s);

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}
```

### AdjustTrajectoryWhichStartsFromCurrentPos()

brief:如果从当前车辆位置开始而不是从上游规划起始点，则调整轨迹

插入规划初始点是一种野蛮的方式，一种优雅的方式是绕过轨迹拼接逻辑，或者使用从轨迹拼接规划初始点来计算最开始的轨迹

问题1：上游规划起始点指的是什么，是车辆当前位置距离上次规划轨迹中最近位置的点或者插值点吗？

```cpp
bool ReferenceLineInfo::AdjustTrajectoryWhichStartsFromCurrentPos(
    const common::TrajectoryPoint& planning_start_point,
    const std::vector<common::TrajectoryPoint>& trajectory,
    DiscretizedTrajectory* adjusted_trajectory) {
  ACHECK(adjusted_trajectory != nullptr);
  // find insert index by check heading
  static constexpr double kMaxAngleDiff = M_PI_2;
  const double start_point_heading = planning_start_point.path_point().theta();
  const double start_point_x = planning_start_point.path_point().x();
  const double start_point_y = planning_start_point.path_point().y();
  const double start_point_relative_time = planning_start_point.relative_time();

  int insert_idx = -1;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    // skip trajectory_points early than planning_start_point
    if (trajectory[i].relative_time() <= start_point_relative_time) {
      continue;
    }

    const double cur_point_x = trajectory[i].path_point().x();
    const double cur_point_y = trajectory[i].path_point().y();
    const double tracking_heading =
        std::atan2(cur_point_y - start_point_y, cur_point_x - start_point_x);
    if (std::fabs(common::math::AngleDiff(start_point_heading,
                                          tracking_heading)) < kMaxAngleDiff) {
      insert_idx = i;
      break;
    }
  }
  if (insert_idx == -1) {
    AERROR << "All points are behind of planning init point";
    return false;
  }

  DiscretizedTrajectory cut_trajectory(trajectory);
  cut_trajectory.erase(cut_trajectory.begin(),
                       cut_trajectory.begin() + insert_idx);
  cut_trajectory.insert(cut_trajectory.begin(), planning_start_point);

  // In class TrajectoryStitcher, the stitched point which is also the planning
  // init point is supposed have one planning_cycle_time ahead respect to
  // current timestamp as its relative time. So the relative timelines
  // of planning init point and the trajectory which start from current
  // position(relative time = 0) are the same. Therefore any conflicts on the
  // relative time including the one below should return false and inspected its
  // cause.
  if (cut_trajectory.size() > 1 && cut_trajectory.front().relative_time() >=
                                       cut_trajectory[1].relative_time()) {
    AERROR << "planning init point relative_time["
           << cut_trajectory.front().relative_time()
           << "] larger than its next point's relative_time["
           << cut_trajectory[1].relative_time() << "]";
    return false;
  }

  // In class TrajectoryStitcher, the planing_init_point is set to have s as 0,
  // so adjustment is needed to be done on the other points
  double accumulated_s = 0.0;
  for (size_t i = 1; i < cut_trajectory.size(); ++i) {
    const auto& pre_path_point = cut_trajectory[i - 1].path_point();
    auto* cur_path_point = cut_trajectory[i].mutable_path_point();
    accumulated_s += std::sqrt((cur_path_point->x() - pre_path_point.x()) *
                                   (cur_path_point->x() - pre_path_point.x()) +
                               (cur_path_point->y() - pre_path_point.y()) *
                                   (cur_path_point->y() - pre_path_point.y()));
    cur_path_point->set_s(accumulated_s);
  }

  // reevaluate relative_time to make delta t the same
  adjusted_trajectory->clear();
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
  for (double cur_rel_time = cut_trajectory.front().relative_time();
       cur_rel_time <= cut_trajectory.back().relative_time();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution)) {
    adjusted_trajectory->AppendTrajectoryPoint(
        cut_trajectory.Evaluate(cur_rel_time));
  }
  return true;
}
```

### IsChangeLanePath()

brief:检查当前参考线是否为变道参考线，即ADC的当前位置不在当前参考线上

```cpp
bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}
```

### IsNeighborLanePath()

brief:检查当前参考线是否是车辆当前位置的邻居

```cpp
bool ReferenceLineInfo::IsNeighborLanePath() const {
  return Lanes().IsNeighborSegment();
}
```

### ExportEngageAdvice()

```cpp
void ReferenceLineInfo::ExportEngageAdvice(
    EngageAdvice* engage_advice, PlanningContext* planning_context) const {
  static EngageAdvice prev_advice;
  static constexpr double kMaxAngleDiff = M_PI / 6.0;

  bool engage = false;
  if (!IsDrivable()) {
    prev_advice.set_reason("Reference line not drivable");
  } else if (!is_on_reference_line_) {
    const auto& scenario_type =
        planning_context->planning_status().scenario().scenario_type();
    if (scenario_type == ScenarioConfig::PARK_AND_GO || IsChangeLanePath()) {
      // note: when is_on_reference_line_ is FALSE
      //   (1) always engage while in PARK_AND_GO scenario
      //   (2) engage when "ChangeLanePath" is picked as Drivable ref line
      //       where most likely ADC not OnLane yet
      engage = true;
    } else {
      prev_advice.set_reason("Not on reference line");
    }
  } else {
    // check heading
    auto ref_point =
        reference_line_.GetReferencePoint(adc_sl_boundary_.end_s());
    if (common::math::AngleDiff(vehicle_state_.heading(), ref_point.heading()) <
        kMaxAngleDiff) {
      engage = true;
    } else {
      prev_advice.set_reason("Vehicle heading is not aligned");
    }
  }

  if (engage) {
    if (vehicle_state_.driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      // READY_TO_ENGAGE when in non-AUTO mode
      prev_advice.set_advice(EngageAdvice::READY_TO_ENGAGE);
    } else {
      // KEEP_ENGAGED when in AUTO mode
      prev_advice.set_advice(EngageAdvice::KEEP_ENGAGED);
    }
    prev_advice.clear_reason();
  } else {
    if (prev_advice.advice() != EngageAdvice::DISALLOW_ENGAGE) {
      prev_advice.set_advice(EngageAdvice::PREPARE_DISENGAGE);
    }
  }
  engage_advice->CopyFrom(prev_advice);
}
```

### TargetLaneId()

```cpp
std::list<hdmap::Id> ReferenceLineInfo::TargetLaneId() const {
  std::list<hdmap::Id> lane_ids;
  for (const auto& lane_seg : lanes_) {
    lane_ids.push_back(lane_seg.lane->id());
  }
  return lane_ids;
}
```

### ExportDecision()

```cpp
void ReferenceLineInfo::ExportDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  MakeDecision(decision_result, planning_context);
  ExportVehicleSignal(decision_result->mutable_vehicle_signal());
  auto* main_decision = decision_result->mutable_main_decision();
  if (main_decision->has_stop()) {
    main_decision->mutable_stop()->set_change_lane_type(
        Lanes().PreviousAction());
  } else if (main_decision->has_cruise()) {
    main_decision->mutable_cruise()->set_change_lane_type(
        Lanes().PreviousAction());
  }
}
```



## 3.public set函数

### SetTrajectory()

```cpp
void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}
```

### SetCost()

```cpp
void SetCost(double cost) { cost_ = cost; }
```

### SetPriorityCost()

```cpp
void SetPriorityCost(double cost) { priority_cost_ = cost; }
```

### SetLatticeStopPoint()

函数功能：用于Lattice planner的速度规划目标

```cpp
void ReferenceLineInfo::SetLatticeStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}
```

### SetLatticeCruiseSpeed()

函数功能：用于Lattice planner的速度规划目标

```cpp
void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
}
```

### SetCruiseSpeed()

```cpp
void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }
```

### SetDrivable()

brief：设置车辆是否可以沿着这条参考线行驶，如果参考线正常，规划器需要将此值设置为true

```cpp
void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }
```

### SetJunctionRightOfWay()

```cpp
void ReferenceLineInfo::SetJunctionRightOfWay(const double junction_s,
                                              const bool is_protected) const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s)) {
      junction_right_of_way_map_[overlap.object_id] = is_protected;
    }
  }
}
```

### SetOffsetToOtherReferenceLine()

```cpp
void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
    }
```

### SetCandidatePathBoundaries()

```cpp
void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}
```

### SetCandidatePathData()

```cpp
void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}
```

### SetBlockingObstacle()

```cpp
void ReferenceLineInfo::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = path_decision_.Find(blocking_obstacle_id);
}
```

### set_is_path_lane_borrow()

```cpp
void set_is_path_lane_borrow(const bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }
```

### set_is_on_reference_line()

```cpp
void set_is_on_reference_line() { is_on_reference_line_ = true; }
```

### SetPriority()

```cpp
void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }
```

### set_trajectory_type()

```cpp
void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }
```

### SetTurnSignal()

```cpp
void ReferenceLineInfo::SetTurnSignal(
    const VehicleSignal::TurnSignal& turn_signal) {
  vehicle_signal_.set_turn_signal(turn_signal);
}
```

### SetEmergencyLight()

```cpp
void ReferenceLineInfo::SetEmergencyLight() {
  vehicle_signal_.set_emergency_light(true);
}
```

### set_path_reusable()

```cpp
void set_path_reusable(const bool path_reusable) {
    path_reusable_ = path_reusable;
  }
```



## 4.public查询函数

### vehicle_state() 

```cpp
const common::VehicleState& vehicle_state() const { return vehicle_state_; }
```

### path_decision()

```cpp
PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }
```

### path_decision()

```cpp
const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}
```

### reference_line()

```cpp
const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}
```

### mutable_reference_line()

```cpp
ReferenceLine* ReferenceLineInfo::mutable_reference_line() {
  return &reference_line_;
}
```

### trajectory()

```cpp
const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}
```

### Cost()

```cpp
 double Cost() const { return cost_; }
```

### PriorityCost()

```cpp
double PriorityCost() const { return priority_cost_; }
```

### planning_target()

```cpp
const PlanningTarget& planning_target() const { return planning_target_; }
```

### GetCruiseSpeed()

```cpp
double ReferenceLineInfo::GetCruiseSpeed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}
```

### mutable_debug()

```cpp
planning_internal::Debug* mutable_debug() { return &debug_; }
```

### debug()

```cpp
const planning_internal::Debug& debug() const { return debug_; }
```

### mutable_latency_stats()

```cpp
LatencyStats* mutable_latency_stats() { return &latency_stats_; }
```

### latency_stats()

```cpp
const LatencyStats& latency_stats() const { return latency_stats_; }
```

### path_data()

```cpp
const PathData& ReferenceLineInfo::path_data() const { return path_data_; }
```

### fallback_path_data()

```cpp
const PathData& ReferenceLineInfo::fallback_path_data() const {
  return fallback_path_data_;
}
```

### speed_data()

```cpp
const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }
```

### mutable_path_data()

```cpp
PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }
```

### mutable_fallback_path_data()

```cpp
PathData* ReferenceLineInfo::mutable_fallback_path_data() {
  return &fallback_path_data_;
}
```

### mutable_speed_data()

```cpp
SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }
```

### rss_info()

```cpp
const RSSInfo& ReferenceLineInfo::rss_info() const { return rss_info_; }
```

### mutable_rss_info()

```cpp
RSSInfo* ReferenceLineInfo::mutable_rss_info() { return &rss_info_; }
```





### AdcSlBoundary()

```cpp
const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return adc_sl_boundary_;
}
```

### PathSpeedDebugString()

```cpp
std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return absl::StrCat("path_data:", path_data_.DebugString(),
                      "speed_data:", speed_data_.DebugString());
}
```

### IsDrivable()

```cpp
bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }
```

### Lanes()

```cpp
const hdmap::RouteSegments& ReferenceLineInfo::Lanes() const { return lanes_; }
```

### GetRightOfWayStatus()

```cpp
ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (overlap.end_s < adc_sl_boundary_.start_s()) {
      junction_right_of_way_map_.erase(overlap.object_id);
    } else if (WithinOverlap(overlap, adc_sl_boundary_.end_s())) {
      auto is_protected = junction_right_of_way_map_[overlap.object_id];
      if (is_protected) {
        return ADCTrajectory::PROTECTED;
      }
    }
  }
  return ADCTrajectory::UNPROTECTED;
}
```

### GetPathTurnType()

```cpp
hdmap::Lane::LaneTurn ReferenceLineInfo::GetPathTurnType(const double s) const {
  const double forward_buffer = 20.0;
  double route_s = 0.0;
  for (const auto& seg : Lanes()) {
    if (route_s > s + forward_buffer) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < s) {
      continue;
    }
    const auto& turn_type = seg.lane->lane().turn();
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      return turn_type;
    }
  }

  return hdmap::Lane::NO_TURN;
}
```

### GetIntersectionRightofWayStatus()

```cpp
bool ReferenceLineInfo::GetIntersectionRightofWayStatus(
    const hdmap::PathOverlap& pnc_junction_overlap) const {
  if (GetPathTurnType(pnc_junction_overlap.start_s) != hdmap::Lane::NO_TURN) {
    return false;
  }

  // TODO(all): iterate exits of intersection to check/compare speed-limit
  return true;
}
```

### OffsetToOtherReferenceLine()

```cpp
double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
```

### GetCandidatePathBoundaries()

```cpp
const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}
```



### GetCandidatePathData()

```cpp
const std::vector<PathData>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}
```

### GetBlockingObstacle()

```cpp
Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }
```

### is_path_lane_borrow()

```cpp
bool is_path_lane_borrow() const { return is_path_lane_borrow_; }
```

### GetPriority()

```cpp
uint32_t GetPriority() const { return reference_line_.GetPriority(); }
```

### trajectory_type()

```cpp
ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }
```

### mutable_st_graph_data()

```cpp
  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }
```

### st_graph_data()

```cpp
  const StGraphData& st_graph_data() { return st_graph_data_; }
```

### enum OverlapType 

brief:不同场景可以处理的不同类型的重叠

```cpp
enum OverlapType {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  };
```

### FirstEncounteredOverlaps()

```cpp
const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }
```

### GetPnCJunction()

```cpp
int ReferenceLineInfo::GetPnCJunction(
    const double s, hdmap::PathOverlap* pnc_junction_overlap) const {
  CHECK_NOTNULL(pnc_junction_overlap);
  const std::vector<hdmap::PathOverlap>& pnc_junction_overlaps =
      reference_line_.map_path().pnc_junction_overlaps();

  static constexpr double kError = 1.0;  // meter
  for (const auto& overlap : pnc_junction_overlaps) {
    if (s >= overlap.start_s - kError && s <= overlap.end_s + kError) {
      *pnc_junction_overlap = overlap;
      return 1;
    }
  }
  return 0;
}
```

### GetAllStopDecisionSLPoint()

```cpp
std::vector<common::SLPoint> ReferenceLineInfo::GetAllStopDecisionSLPoint()
    const {
  std::vector<common::SLPoint> result;
  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }
    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);
    if (stop_line_sl.s() <= 0 || stop_line_sl.s() >= reference_line_.Length()) {
      continue;
    }
    result.push_back(stop_line_sl);
  }

  // sort by s
  if (!result.empty()) {
    std::sort(result.begin(), result.end(),
              [](const common::SLPoint& a, const common::SLPoint& b) {
                return a.s() < b.s();
              });
  }

  return result;
}
```

path_reusable()

```cpp
  bool path_reusable() const { return path_reusable_; }
```

# 5.private成员函数

### InitFirstOverlaps()

```cpp
void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_.map_path();
  // clear_zone
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.emplace_back(CLEAR_AREA, clear_area_overlap);
  }

  // crosswalk
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.emplace_back(CROSSWALK, crosswalk_overlap);
  }

  // pnc_junction
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.emplace_back(PNC_JUNCTION, pnc_junction_overlap);
  }

  // signal
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.emplace_back(SIGNAL, signal_overlap);
  }

  // stop_sign
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(STOP_SIGN, stop_sign_overlap);
  }

  // yield_sign
  hdmap::PathOverlap yield_sign_overlap;
  if (GetFirstOverlap(map_path.yield_sign_overlaps(), &yield_sign_overlap)) {
    first_encounter_overlaps_.emplace_back(YIELD_SIGN, yield_sign_overlap);
  }

  // sort by start_s
  if (!first_encounter_overlaps_.empty()) {
    std::sort(first_encounter_overlaps_.begin(),
              first_encounter_overlaps_.end(),
              [](const std::pair<OverlapType, hdmap::PathOverlap>& a,
                 const std::pair<OverlapType, hdmap::PathOverlap>& b) {
                return a.second.start_s < b.second.start_s;
              });
  }
}
```



## CheckChangeLane() const,没有找到这个函数的实现

### SetTurnSignalBasedOnLaneTurnType()

```cpp
void ReferenceLineInfo::SetTurnSignalBasedOnLaneTurnType(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  if (vehicle_signal->has_turn_signal() &&
      vehicle_signal->turn_signal() != VehicleSignal::TURN_NONE) {
    return;
  }
  vehicle_signal->set_turn_signal(VehicleSignal::TURN_NONE);

  // Set turn signal based on lane-change.
  if (IsChangeLanePath()) {
    if (Lanes().PreviousAction() == routing::ChangeLaneType::LEFT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    } else if (Lanes().PreviousAction() == routing::ChangeLaneType::RIGHT) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    }
    return;
  }

  // Set turn signal based on lane-borrow.
  if (path_data_.path_label().find("left") != std::string::npos) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
    return;
  }
  if (path_data_.path_label().find("right") != std::string::npos) {
    vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
    return;
  }

  // Set turn signal based on lane's turn type.
  double route_s = 0.0;
  const double adc_s = adc_sl_boundary_.end_s();
  for (const auto& seg : Lanes()) {
    if (route_s > adc_s + FLAGS_turn_signal_distance) {
      break;
    }
    route_s += seg.end_s - seg.start_s;
    if (route_s < adc_s) {
      continue;
    }
    const auto& turn = seg.lane->lane().turn();
    if (turn == hdmap::Lane::LEFT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      break;
    } else if (turn == hdmap::Lane::RIGHT_TURN) {
      vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      break;
    } else if (turn == hdmap::Lane::U_TURN) {
      // check left or right by geometry.
      auto start_xy =
          PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.start_s));
      auto middle_xy = PointFactory::ToVec2d(
          seg.lane->GetSmoothPoint((seg.start_s + seg.end_s) / 2.0));
      auto end_xy = PointFactory::ToVec2d(seg.lane->GetSmoothPoint(seg.end_s));
      auto start_to_middle = middle_xy - start_xy;
      auto start_to_end = end_xy - start_xy;
      if (start_to_middle.CrossProd(start_to_end) < 0) {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_RIGHT);
      } else {
        vehicle_signal->set_turn_signal(VehicleSignal::TURN_LEFT);
      }
      break;
    }
  }
}
```

### ExportVehicleSignal()

```cpp
void ReferenceLineInfo::ExportVehicleSignal(
    common::VehicleSignal* vehicle_signal) const {
  CHECK_NOTNULL(vehicle_signal);
  *vehicle_signal = vehicle_signal_;
  SetTurnSignalBasedOnLaneTurnType(vehicle_signal);
}
```

### IsIrrelevantObstacle()

```cpp
bool ReferenceLineInfo::IsIrrelevantObstacle(const Obstacle& obstacle) {
  if (obstacle.IsCautionLevelObstacle()) {
    return false;
  }
  // if adc is on the road, and obstacle behind adc, ignore
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();
  if (obstacle_boundary.end_s() > reference_line_.Length()) {
    return true;
  }
  if (is_on_reference_line_ && !IsChangeLanePath() &&
      obstacle_boundary.end_s() < adc_sl_boundary_.end_s() &&
      (reference_line_.IsOnLane(obstacle_boundary) ||
       obstacle_boundary.end_s() < 0.0)) {  // if obstacle is far backward
    return true;
  }
  return false;
}
```

### MakeDecision()

```cpp
void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result,
                                     PlanningContext* planning_context) const {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();

  // cruise by default
  decision_result->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result, planning_context);
  SetObjectDecisions(decision_result->mutable_object_decision());
}
```

### MakeMainStopDecision()

```cpp
int ReferenceLineInfo::MakeMainStopDecision(
    DecisionResult* decision_result) const {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& object_decision = obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line_.XYToSL(stop_point, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}
```

### MakeMainMissionCompleteDecision()

```cpp
void ReferenceLineInfo::MakeMainMissionCompleteDecision(
    DecisionResult* decision_result, PlanningContext* planning_context) const {
  if (!decision_result->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION &&
      main_stop.reason_code() != STOP_REASON_PULL_OVER) {
    return;
  }
  const auto& adc_pos = adc_planning_point_.path_point();
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
      FLAGS_destination_check_distance) {
    return;
  }

  auto mission_complete =
      decision_result->mutable_main_decision()->mutable_mission_complete();
  if (ReachedDestination()) {
    planning_context->mutable_planning_status()
        ->mutable_destination()
        ->set_has_passed_destination(true);
  } else {
    mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
    mission_complete->set_stop_heading(main_stop.stop_heading());
  }
}
```

### MakeEStopDecision()

```cpp
void ReferenceLineInfo::MakeEStopDecision(
    DecisionResult* decision_result) const {
  decision_result->Clear();

  MainEmergencyStop* main_estop =
      decision_result->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result->mutable_object_decision();
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}
```

### SetObjectDecisions()

```cpp
void ReferenceLineInfo::SetObjectDecisions(
    ObjectDecisions* object_decisions) const {
  for (const auto obstacle : path_decision_.obstacles().Items()) {
    if (!obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (obstacle->HasLateralDecision() && !obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LateralDecision());
    }
    if (obstacle->HasLongitudinalDecision() &&
        !obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          obstacle->LongitudinalDecision());
    }
  }
}
```

### AddObstacleHelper()

```cpp
bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle.get()) != nullptr;
}
```

### GetFirstOverlap()

```cpp
bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);
  const double start_s = adc_sl_boundary_.end_s();
  static constexpr double kMaxOverlapRange = 500.0;
  double overlap_min_s = kMaxOverlapRange;

  auto overlap_min_s_iter = path_overlaps.end();
  for (auto iter = path_overlaps.begin(); iter != path_overlaps.end(); ++iter) {
    if (iter->end_s < start_s) {
      continue;
    }
    if (overlap_min_s > iter->start_s) {
      overlap_min_s_iter = iter;
      overlap_min_s = iter->start_s;
    }
  }

  // Ensure that the path_overlaps is not empty.
  if (overlap_min_s_iter != path_overlaps.end()) {
    *path_overlap = *overlap_min_s_iter;
  }

  return overlap_min_s < kMaxOverlapRange;
}
```



# 再看看有啥私有成员变量

static std::unordered_map<std::string, bool> junction_right_of_way_map_;

const common::VehicleState vehicle_state_;

const common::TrajectoryPoint adc_planning_point_;

ReferenceLine reference_line_; 

double cost_ = 0.0;   //这是评价这条参考线优劣的数字，越低越好

bool is_drivable_ = true;

PathDecision path_decision_;

Obstacle* blocking_obstacle_;

std::vector<PathBoundary> candidate_path_boundaries_;

std::vector<PathData> candidate_path_data_;

PathData path_data_;

PathData fallback_path_data_;

SpeedData speed_data_;

DiscretizedTrajectory discretized_trajectory_;

RSSInfo rss_info_;

SLBoundary adc_sl_boundary_;  //缝合点(stitching point)(计划轨迹起点)相对于参考线的SL边界

planning_internal::Debug debug_;

LatencyStats latency_stats_;

hdmap::RouteSegments lanes_;

bool is_on_reference_line_ = false;

bool is_path_lane_borrow_ = false;

ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

double offset_to_other_reference_line_ = 0.0;

double priority_cost_ = 0.0;

PlanningTarget planning_target_;

ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

std::vector<std::pair<OverlapType, hdmap::PathOverlap>>  first_encounter_overlaps_;  //沿车辆前方参考线第一次遇到重叠

StGraphData st_graph_data_;  //speed_bounds_decider生成的数据，用于为不同的st优化器构建st_graph

common::VehicleSignal vehicle_signal_;

double cruise_speed_ = 0.0;

bool path_reusable_ = false;

DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);