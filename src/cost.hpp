#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H
#include <iostream>
#include <vector>
#include <functional>
#include "vehicle.hpp"
#include "trajectory.hpp"
#include "statemachine.hpp"
#include "helpers.h"
#include "constants.hpp"

class CostEvaluator
{

    Vehicle &m_ego;
    std::vector<Vehicle> &m_others;

    std::vector<std::function<double(const Trajectory &, const State)>> m_cost_funtion_list = {
        std::bind(&CostEvaluator::exceeds_speed_limit_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::stays_on_road_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::distance_to_center_of_lane_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::distance_to_vehicle_ahead_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::efficient_speed_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::efficient_lane_speed_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::difference_in_speed_to_vehicle_ahead_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::difference_in_max_acceleration_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::difference_in_average_acceleration_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::frontal_collision_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CostEvaluator::rear_and_lateral_collision_cost_function, this, std::placeholders::_1, std::placeholders::_2)
        // std::bind(&CostEvaluator::distance_to_goal_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        // std::bind(&CostEvaluator::difference_in_max_jerk_cost_function, this, std::placeholders::_1, std::placeholders::_2),
        // std::bind(&CostEvaluator::difference_in_average_jerk_cost_function, this, std::placeholders::_1, std::placeholders::_2)
    };

    std::vector<double> m_cost_function_weights = {SPEED_LIMIT_WEIGHT, STAYS_ON_ROAD_WEIGHT, DISTANCE_TO_CENTER_WEIGHT, DISTANCE_TO_VEHICLE_AHEAD_WEIGHT,
                                                   EFFICIENT_SPEED_WEIGHT, EFFICIENT_LANE_SPEED_WEIGHT, SPEED_DIFFERENCE_TO_VEHICLE_AHEAD_WEIGHT, MAX_ACCELERATION_WEIGHT, TOTAL_ACCELERATION_WEIGHT,
                                                   FRONTAL_COLLISION_WEIGHT, REAR_COLLISION_WEIGHT /*, DISTANCE_TO_GOAL_WEIGHT, MAX_JERK_WEIGHT, TOTAL_JERK_WEIGHT,*/};

    double exceeds_speed_limit_cost_function(Trajectory const &trajectory, const State state) const;

    double distance_to_center_of_lane_cost_function(Trajectory const &trajectory, const State state) const;

    double stays_on_road_cost_function(Trajectory const &trajectory, const State state) const;

    double distance_to_vehicle_ahead_cost_function(Trajectory const &trajectory, const State state) const;

    double efficient_speed_cost_function(Trajectory const &trajectory, const State state) const;

    double efficient_lane_speed_cost_function(Trajectory const &trajectory, const State state) const;

    double difference_in_speed_to_vehicle_ahead_cost_function(Trajectory const &trajectory, const State state) const;

    double difference_in_max_acceleration_cost_function(Trajectory const &trajectory, const State state) const;

    double difference_in_average_acceleration_cost_function(Trajectory const &trajectory, const State state) const;

    double frontal_collision_cost_function(Trajectory const &trajectory, const State state) const;

    double rear_and_lateral_collision_cost_function(Trajectory const &trajectory, const State state) const;

    // Cost functions that could be useful

    // double distance_to_goal_cost(Trajectory const &trajectory,
    //                          const State state);

    // double difference_in_max_jerk_cost_function(Trajectory const &trajectory,
    //                                         const State state);

    // double difference_in_average_jerk_cost_function(Trajectory const &trajectory,
    //                                         const State state);

public:

    CostEvaluator(Vehicle &ego, std::vector<Vehicle> &others);

    double evaluate_cost(Trajectory const &trajectory, const State state) const;
};
#endif