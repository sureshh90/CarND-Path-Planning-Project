#include "trajectorygenerator.hpp"
#include "jerkminimiser.hpp"
#include "vehicle.hpp"

TrajectoryGenerator::TrajectoryGenerator(Vehicle &ego, Trajectory &current_trajectory)
    : m_ego(ego), m_current_trajectory(current_trajectory){};

std::pair<JmtState, JmtState> TrajectoryGenerator::get_target_states(const State &state,
                                                                     int retention_path_last_index) const

{
  double target_s = 0.0;
  double target_s_vel = 0.0;
  double target_s_acc = 0.0;

  double target_d = 0.0;
  double target_d_vel = 0.0;
  double target_d_acc = 0.0;

  double speed_at_index = m_current_trajectory.s_dot_vector[retention_path_last_index];

  switch (state.s_state)
  {
  case LongitudinalState::MAINTAIN:
    target_s_vel = speed_at_index;
    break;
  case LongitudinalState::ACCELERATE:
    if (speed_at_index == 0)
    {
      target_s_vel = 3.0; // Assign something below 10 m/s to keep the max acceleration under 10 m/s^2
    }
    else
    {
      double factor = 1.05;

      if (speed_at_index < 10)
      {
        factor = 1.1; // high acceleration at lower speeds
      }

      target_s_vel = speed_at_index * factor;
    }
    break;
  case LongitudinalState::DECELERATE:
    target_s_vel = speed_at_index * 0.85;
    break;
  case LongitudinalState::HARD_DECELERATE:
    target_s_vel = speed_at_index * 0.7;
    break;
  }

  if (target_s_vel >= MAX_SPEED)
  {
    target_s_vel = MAX_SPEED; // Limit the speed
  }

  double d_at_index = m_current_trajectory.d_vector[retention_path_last_index];

  switch (state.d_state)
  {
  case LateralState::KEEP_LANE:
    target_d = get_lane_center_frenet(state.current_lane);
    break;
  case LateralState::PREPARE_CHANGE_LANE_LEFT:
    target_d = get_lane_center_frenet(state.current_lane);
    break;
  case LateralState::PREPARE_CHANGE_LANE_RIGHT:
    target_d = get_lane_center_frenet(state.current_lane);
    break;
  case LateralState::CHANGE_LANE_LEFT:
    target_d = get_lane_center_frenet(state.future_lane);
    break;
  case LateralState::CHANGE_LANE_RIGHT:
    target_d = get_lane_center_frenet(state.future_lane);
    break;
  }

  target_s = (m_current_trajectory.s_vector[retention_path_last_index] + target_s_vel * TIME_INTERVAL);

  JmtState target_s_state = {target_s, target_s_vel, target_s_acc};
  JmtState target_d_state = {target_d, target_d_vel, target_d_acc};

  return std::make_pair(target_s_state, target_d_state);
}

std::pair<JmtState, JmtState> TrajectoryGenerator::get_start_states(Trajectory const &new_trajectory) const
{
  int trajectory_size = new_trajectory.size();

  double last_s = 0.0;
  double last_s_vel = 0.0;
  double last_s_acc = 0.0;

  double last_d = 0.0;
  double last_d_vel = 0.0;
  double last_d_acc = 0.0;

  if (trajectory_size > 0)
  {
    last_s = new_trajectory.s_vector.back();
    last_d = new_trajectory.d_vector.back();
    last_s_vel = new_trajectory.s_dot_vector.back();
    last_d_vel = new_trajectory.d_dot_vector.back();
    last_s_acc = new_trajectory.s_dot_dot_vector.back();
    last_d_acc = new_trajectory.d_dot_dot_vector.back();
  }

  JmtState start_s_state = {last_s, last_s_vel, last_s_acc};

  JmtState start_d_state = {last_d, last_d_vel, last_d_acc};

  return std::make_pair(start_s_state, start_d_state);
}

void TrajectoryGenerator::append_new_points(std::pair<JmtState, JmtState> start_states, std::pair<JmtState, JmtState> target_states,
                                            Trajectory &trajectory, const State &state)
{
  vector<double> coeffs_s = JMT(start_states.first, target_states.first, TIME_INTERVAL);
  vector<double> coeffs_d = JMT(start_states.second, target_states.second, TIME_INTERVAL);

  int total_points = TIME_INTERVAL / SIMULATOR_DELTA_T;
  int remaining_points = total_points - trajectory.size();

  Map &map = Map::get_map_instance();

  double last_x = trajectory.x_vector[trajectory.size() - 1];
  double last_y = trajectory.y_vector[trajectory.size() - 1];

  std::vector<std::function<double(double)>> derivatives_s = get_function_and_its_derivatives(coeffs_s);
  std::vector<std::function<double(double)>> derivatives_d = get_function_and_its_derivatives(coeffs_d);

  for (int i = 0; i < remaining_points; ++i)
  {
    double t = SIMULATOR_DELTA_T * (i + 1);

    double s_t = derivatives_s[0](t);
    double s_t_dot = derivatives_s[1](t);
    double s_t_dot_dot = derivatives_s[2](t);
    double s_jerk = derivatives_s[3](t);

    double d_t = derivatives_d[0](t);
    double d_t_dot = derivatives_d[1](t);
    double d_t_dot_dot = derivatives_d[2](t);
    double d_jerk = derivatives_d[3](t);

    // TODO: Remove after debugging
    // if ((s_t > 2990 && s_t < 3020))
    // {
      
    //   if (count == 0.0)
    //   {
    //     count = d_t;
    //   }
    //   if ((d_t < count - 1.0) || (d_t > count + 1.0))
    //     std::cout << "The suspected d value is " << d_t << std::endl;
    // }

    // if ((s_t > 4920 && s_t < 4950))
    // {
    //   if (count_1 == 0.0)
    //   {
    //     count_1 = d_t;
    //   }
    //   if ((d_t < count_1 - 1.0) || (d_t > count_1 + 1.0))
    //     std::cout << "The suspected d value is " << d_t << std::endl;
    // }

    vector<double> x_y = Map::get_map_instance().get_spline_XY(s_t, d_t);

    // TODO Check whether this is right
    double yaw = atan2(x_y[1] - last_y, x_y[0] - last_x);

    // Add the point to the trajectory
    trajectory.add_point(x_y[0], x_y[1],
                         s_t, s_t_dot, s_t_dot_dot, s_jerk,
                         d_t, d_t_dot, d_t_dot_dot, d_jerk,
                         yaw);
  }
}

std::vector<Trajectory> TrajectoryGenerator::generate_trajectories(const State &state,
                                                                   int retention_path_last_index,
                                                                   int number_of_trajectories)

{

  std::vector<Trajectory> new_trajectories;

  for (int i = 0; i < number_of_trajectories; i++) // Useful if trajectory perturbation is needed
  {
    // Copy the retention path
    Trajectory new_trajectory = m_current_trajectory.copy_n_upto_index(retention_path_last_index);

    // Get start states
    auto start_states = get_start_states(new_trajectory);

    // Get target states
    auto target_states = get_target_states(state, retention_path_last_index);

    // Add new points to the trajectory
    append_new_points(start_states, target_states, new_trajectory, state);

    // Add the trajectory to the vector
    new_trajectories.push_back(new_trajectory);
  }

  return new_trajectories;
}
