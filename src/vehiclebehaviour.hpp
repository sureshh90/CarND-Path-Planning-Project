#include "statemachine.hpp"
#include "trajectorygenerator.hpp"
#include "trajectory.hpp"
#include "vehicle.hpp"

class VehicleBehaviour
{
    TrajectoryGenerator m_tgen;

    StateMachine m_fsm;

    Vehicle &m_ego;

    std::vector<Vehicle> &m_other_vehicles;

    Trajectory m_current_trajectory;

    bool m_is_start;

public:
    VehicleBehaviour(Vehicle &ego, std::vector<Vehicle> &other_vehicles);
    Trajectory generate_optimal_trajectory(int previous_path_size);
    void update_current_trajectory(Trajectory new_trajectory);
};
