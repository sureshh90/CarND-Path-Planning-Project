#include "vehiclebehaviour.hpp"
#include "statemachine.hpp"
#include "cost.hpp"

VehicleBehaviour::VehicleBehaviour(Vehicle &ego, std::vector<Vehicle> &other_vehicles)
    : m_ego(ego), m_other_vehicles(other_vehicles), m_current_trajectory(Trajectory()), m_tgen(TrajectoryGenerator(m_ego, m_current_trajectory))
{
    m_fsm = StateMachine();
    m_is_start = true;
}

void VehicleBehaviour::update_current_trajectory(Trajectory new_trajectory)
{
    m_current_trajectory = new_trajectory;
}

Trajectory VehicleBehaviour::generate_optimal_trajectory(int previous_path_size)
{

    // std::cout << "Debug: Previous path size " << previous_path_size << std::endl;

    int retention_path_last_index = m_is_start == true ? 0 : (RETAIN_PATH_LENGTH - 1); // Reduce one since it is an index

    if (m_is_start == true)
    {
        // Add initial point to the trajectory
        m_current_trajectory.add_point(m_ego.car_x, m_ego.car_y, m_ego.car_s, 0.0, 0.0, 0.0, m_ego.car_d, 0.0, 0.0, 0.0, m_ego.car_yaw);

        // Update the state machine
        int start_lane = get_lane_index(m_current_trajectory.d_vector.front());
        int finish_lane = get_lane_index(m_current_trajectory.d_vector.back());
        State start_vehicle_state = State(LongitudinalState::ACCELERATE, LateralState::KEEP_LANE, start_lane, finish_lane);
        m_fsm.update_current_fsm_state(start_vehicle_state);
    }

    int used_points = m_current_trajectory.size() - previous_path_size;

    m_is_start = false;

    if (previous_path_size > 0)
    {
        m_current_trajectory.erase_n_at_start(used_points);
    }

    std::vector<State> possible_states = m_fsm.get_next_reachable_states();

    // Initialise variables to be returned
    Trajectory best_trajectory;

    CostEvaluator evaluator(m_ego, m_other_vehicles);
    std::vector<Trajectory> trajectories_vector;

    std::vector<State> states_vector;
    std::vector<double> trajectory_cost_vector;

    std::for_each(possible_states.begin(), possible_states.end(), [&](State const &state)
                  {
                      std::cout << "Debug: State S State " << (int)state.s_state << " D State " << (int)state.d_state << std::endl;
                      // Generate a trajectory for the current state
                      std::vector<Trajectory> all_trajectories = m_tgen.generate_trajectories(state, retention_path_last_index, 1);

                      for_each(all_trajectories.begin(), all_trajectories.end(), [&](Trajectory const &trajectory)
                               {
                                   states_vector.push_back(state);

                                   trajectory_cost_vector.push_back(evaluator.evaluate_cost(trajectory, state));

                                   trajectories_vector.push_back(trajectory);
                               });
                  });

    vector<double>::iterator best_cost = std::min_element(trajectory_cost_vector.begin(), trajectory_cost_vector.end());
    int best_trajectory_index = std::distance(trajectory_cost_vector.begin(), best_cost);
    best_trajectory = trajectories_vector[best_trajectory_index];
    
    // Start: Debugging Info
    std::cout << "-------------------------------------------------------------------------" << std::endl;
    std::cout << "State Selelected S State " << (int)states_vector[best_trajectory_index].s_state << " D State " << (int)states_vector[best_trajectory_index].d_state << std::endl;
    std::cout << "-------------------------------------------------------------------------" << std::endl;
    std::cout << "Cost" << std::endl;
    for (auto i : trajectory_cost_vector)
        std::cout << i << "\t" << std::endl;
    std::cout << "-------------------------------------------------------------------------" << std::endl;
    // End: Debugging Info

    // Update the current state of fsm
    m_fsm.update_current_fsm_state(states_vector[best_trajectory_index]);

    return best_trajectory;
};
