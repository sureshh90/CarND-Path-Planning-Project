#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "constants.hpp"
#include "helpers.h"
#include "map.hpp"
#include "trajectory.hpp"
#include "statemachine.hpp"

typedef std::vector<double> JmtState;

class TrajectoryGenerator
{
  Vehicle &m_ego;

  Trajectory &m_current_trajectory;

  std::pair<JmtState, JmtState> get_target_states(State const &state,
                                                  int retention_path_last_index) const;

  std::pair<JmtState, JmtState> get_start_states(Trajectory const &new_trajectory) const;

  void append_new_points(std::pair<JmtState, JmtState> start_states, std::pair<JmtState, JmtState> target_states,
                         Trajectory &trajectory, const State &state);

public:

  TrajectoryGenerator(Vehicle &ego, Trajectory &current_trajectory);

  std::vector<Trajectory> generate_trajectories(const State &state,
                                                int retention_path_last_index,
                                                int number_of_trajectories);
};

#endif