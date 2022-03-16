# Motion Planning and Decision Making for Autonomous Vehicles
In this project, two major components of a hierarchical planner were implemented for autonomous driving: (1) Behavioral Planner, and (2) Motion Planner. 

The code includes:
- Behavioral planning using Finite State Machines (FSM).
- Static objects collision checking.
- Path and trajectory generation using cubic spirals.
- Best trajectory selection based on a cost function evaluation. 

The cost function used in this code mainly performs a collision check and a proximity check to bring cost higher as we get closer or collide with objects but maintaining a bias to stay closer to the lane center line.

The behavior planner and the motion planner work simultaneously to avoid colliding with static objects (e.g., cars) parked on the side of the road that are invading the traveling lane. In such cases, the vehicle executes a "nudge" or a "lane change" maneuver. Furthermore, the vehcile handles intersections and roundabouts by a "decelration to stop" maneuver followed by a "STOPPING" maneuver. In other cases, the vehicle tracks the centerline on the traveling lane.

To complete this project, `TODO`s were completed in `behavior_planner_FSM.cpp`, `cost_functions.cpp`, `motion_planner.cpp`, `velocity_profile_generator.cpp`, and `planning_params.h`. In the following, each step has been discussed, separately. These codes can be found:
```
./project/starter_files/
```
## Sample Videos
![Planning](./Motion_Planning.gif)

## Implementation of Decision Making Framework in behavior_planner_FSM.cpp
1- **Look ahead:** In `behavior_planner_FSM.cpp`, under `BehaviorPlannerFSM::get_look_ahead_distance` method, a look ahead distance was defined based on the velocity magnitude and acceleration magnitude of the ego car. As mentioned in the code, one way to find a reasonable lookahead distance is to find the distance you will need to come to a stop while traveling at speed V and using a comfortable deceleration.
```
auto look_ahead_distance = velocity_mag * velocity_mag / (2*accel_mag); 

look_ahead_distance =
    std::min(std::max(look_ahead_distance, _lookahead_distance_min),
            _lookahead_distance_max);

```

2- **Goal bheund the stopping point:** In `behavior_planner_FSM.cpp`, under `BehaviorPlannerFSM::state_transition` method, the goal was placed behind the stopping point (i.e the actual goal location) by `_stop_line_buffer`. We need to go back in the opposite direction of the goal/road, thus, we used an the goal heading angle plus 180 degrees.
```
auto ang = goal.rotation.yaw + M_PI;
goal.location.x += _stop_line_buffer * std::cos(ang);
goal.location.y += _stop_line_buffer * std::sin(ang);  
```

3- **Goal speed at stopping point:** the goal speed at stopping point should be equal to zero.In `behavior_planner_FSM.cpp`, under `BehaviorPlannerFSM::state_transition` method: 
```
goal.velocity.x = 0.0;
goal.velocity.y = 0.0;  
goal.velocity.z = 0.0; 
```
4- **Goal speed in nominal state:** other than stopping state, in the nominal state, the goal speed can be defined based on the speed limit. In `behavior_planner_FSM.cpp`, under `BehaviorPlannerFSM::state_transition` method: 
```
goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw);
goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw);
goal.velocity.z = 0;
```
5- **Maintain the same goal when in DECEL_TO_STOP state:** we need to make sure the new goal is the same as the previous goal (`_goal`). That way we maintain the goal at the stop line:
```
goal = _goal;
```
6- **Calculate the distance:** When we teleport, the car is always at speed zero. In this the case, as soon as we enter the DECEL_TO_STOP state, the condition that we are <= `_stop_threshold_speed` is ALWAYS true and we move straight to `STOPPED` state. To solve this issue (since we don't have a motion controller yet), we should use distance instead of speed. We need to make sure the distance to the stopping point is <= `P_STOP_THRESHOLD_DISTANCE`. Therefore, we need to uncomment the line used to calculate the distance in `behavior_planner_FSM.cpp`, under `BehaviorPlannerFSM::state_transition` method: :
```
auto distance_to_stop_sign =
    utils::magnitude(goal.location - ego_state.location);
```
7-**Use ditance rather than speed**: We used distance rather than speed.

8- **move to STOPPED state**: Now that we know we are close or at the stopping point we should change state to "STOPPED":
```
if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {
      _active_maneuver = STOPPED;
      _start_stop_time = std::chrono::high_resolution_clock::now();

    }
```

9- **Maintain the same goal when in STOPPED state**:
```
else if (_active_maneuver == STOPPED) {
    // maintain the same goal when in STOPPED state: Make sure the new goal
    // is the same as the previous goal.
    goal = _goal; 

    ...}
```
10- **Move to FOLLOW_LANE state:** When we are done at the STOPPED state, we move to the FOLLOW_LANE state:

```
if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
    _active_maneuver = FOLLOW_LANE ;  // <- Fix This
}
```

## Path Planning using Cubic Spirals
### planning_params.h
- The aim was to generate offset goals equidistant from both sides of the main (centerline) goal at the correct position and heading following the road direction. It is worth mentioning that too many offset goals (>7) might be unnecessary and computationally very intensive. On the ohter hand too few (<5) and we might not be able to find a feasible and collision free path that might clearly exist. In the `planning_params.h`, we set:
```
#define P_NUM_PATHS 5    
```
- Furthermore, the number of points used to discretize the spiral was set. Note, deciding the resolution of descritization can have a major effect on checking for collisions and visualization. Using too many points leads to a high computation penalty and the visualization will be terribly slow. On the other hand using too few points may lead to missing potential collisions. In addition, using too few points will lead to teleport large distances making it visually not natural. We set the number of points:

```
#define P_NUM_POINTS_IN_SPIRAL 10  
```

### motion_planner.cpp
- The offset goals will be aligned on a perpendiclular line to the heading of the main goal. To get a perpendicular angle, we added 90 degrees to the main
goal heading. Under `MotionPlanner::generate_offset_goals` method:
```
auto yaw = goal_state.rotation.yaw + M_PI_2;
```

- Offset goal locations were calculated. The x and y position of the offset goals were obtained using "offset". The goals should lie on a perpendicular line to the direction of the main goal as calculated above (`yaw`). 
```
  for (int i = 0; i < _num_paths; ++i) {
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset;

    goal_offset.location.x += offset * std::cos(yaw);  
    goal_offset.location.y += offset * std::sin(yaw);
    
    if (valid_goal(goal_state, goal_offset)) {
      goals_offset.push_back(goal_offset);
    }
  }
  ```

## Trajectory Generation
- In `velocity_profile_generator.cpp` under `VelocityProfileGenerator::calc_distance` method, we used one of the common rectilinear accelerated equations of motion to calculate the distance (`d`)traveled while going from an initial velocit (`v_i`) to a final velocity (`v_f`) at a constant acceleration/deceleration (`a`):
```
double VelocityProfileGenerator::calc_distance(const double& v_i,
                                               const double& v_f,
                                               const double& a) const {
  double d{0.0};
  if (std::abs(a) < DBL_EPSILON) {
    d = std::numeric_limits<double>::infinity();
  } else {
    d = (std::pow(v_f, 2) - std::pow(v_i, 2)) / (2.0 * a);  // <- Update
  }
  return d;
}
```

- In `velocity_profile_generator.cpp` under `VelocityProfileGenerator::calc_final_speed` method, we calculated the final speed (`v_f`) based on the initial velocity (`v_i`), travelled distance (`d`), and acceleration/deceleration (`a`):

```
double VelocityProfileGenerator::calc_final_speed(const double& v_i,
                                                  const double& a,
                                                  const double& d) const {
  double v_f{0.0};
  double disc = std::pow(v_i, 2) + 2.0 * d * a;  // <- Fix this
  if (disc <= 0.0) {
    v_f = 0.0;
  } else if (disc == std::numeric_limits<double>::infinity() ||
             std::isnan(disc)) {
    v_f = std::numeric_limits<double>::infinity();
  } else {
    v_f = std::sqrt(disc);
  }
  return v_f;
}

```
## Trajectory Selection
- In `cost_functions.cpp` under `collision_circles_cost_spiral` function, the center of the circles representing the ego car were placed:

```
auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);
```
- In addition the distance from the center of the circles representing the ego cat to the center of the circles representing the obstacles/actors were obtained:
```
auto diff_center_x = circle_center_x - actor_center_x;
auto diff_center_y = circle_center_y - actor_center_y;
double dist = std::sqrt( std::pow(diff_center_x, 2) + std::pow(diff_center_y, 2) );
```
A collision was detected if the distance `dist` between the center of a circle representing the ego car and a center of a circle representing an actor was smaller than the sum of the circles' radii:
```
collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
```