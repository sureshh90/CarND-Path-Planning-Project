#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include "cost.hpp"
#include "vehicle.hpp"
#include "trajectory.hpp"
#include "statemachine.hpp"
#include "constants.hpp"

#include "helpers.h"

CostEvaluator::CostEvaluator(Vehicle &ego, std::vector<Vehicle> &others) : m_ego(ego), m_others(others){};

/**
 * @brief Evaluates the cost of different functions and returns the accumulated cost.
 */
double CostEvaluator::evaluate_cost(Trajectory const &trajectory,
                                    const State state) const
{

    double total_cost = 0.0;

    for (int i = 0; i < m_cost_funtion_list.size(); ++i)
    {
        double cost = m_cost_function_weights[i] * m_cost_funtion_list[i](trajectory, state);
        total_cost += cost;
    }

    return total_cost;
}

/**
 * @brief Penalises trajectories that exceeds the MAX_SPEED limit.
 */
double CostEvaluator::exceeds_speed_limit_cost_function(Trajectory const &trajectory,
                                                        const State state) const
{
    double max_speed = trajectory.get_max_speed();
    return (max_speed > MAX_SPEED) ? 1 : 0;
}

/**
 * @brief Penalises trajectories that attempts to go outside the bounds of the road (right hand lanes).
 */
double CostEvaluator::stays_on_road_cost_function(Trajectory const &trajectory,
                                                  const State state) const
{
    const auto minmax = std::minmax_element((trajectory.d_vector).begin(), (trajectory.d_vector).end());
    return (*(minmax.first) < 0.0 && *(minmax.second) > 12.0) ? 1 : 0;
}

/**
 * @brief Penalises trajectories that veers to the sides of the current lane.
 */
double CostEvaluator::distance_to_center_of_lane_cost_function(Trajectory const &trajectory,
                                                               const State state) const
{
    double final_d = trajectory.d_vector[trajectory.d_vector.size() - 1];
    double d_sum = std::accumulate(trajectory.d_vector.begin(), trajectory.d_vector.end(), 0.0);
    double d_average = d_sum / trajectory.d_vector.size();
    int lane = get_lane_index(final_d);
    int original_lane_center = get_lane_center_frenet(lane);

    double diff = (original_lane_center - d_average);
    return logistic(diff / original_lane_center);
}

/**
 * @brief  Penalises trajectories that goes too close to the vehicle in front.
 * Note: The goal is not to go close to vehicle ahead. 
 * If the distance is less than 20 m then the trajectory is penalised heavily.
 */
double CostEvaluator::distance_to_vehicle_ahead_cost_function(Trajectory const &trajectory,
                                                              const State state) const
{
    if (state.d_state == LateralState::CHANGE_LANE_LEFT || state.d_state == LateralState::CHANGE_LANE_RIGHT)
    {
        return 0; // Don't check during lane change since it might lead to indecisive driving. (change)
    }

    // Find all vehicles ahead on the future lane
    std::vector<Vehicle> vehicles_ahead = m_ego.get_vehicles_in_front(m_others, state.future_lane);

    if (vehicles_ahead.size() == 0)
    {
        return 0.0; // Don't penalise if we dont have vehicles in front
    }

    double min_distance = REGION_OF_INTEREST_METERS; // Define the Region of Interest (ROI)
    for (const Vehicle &v : vehicles_ahead)
    {
        double dist = distance(m_ego.car_x, m_ego.car_y, v.car_x, v.car_y);
        if (dist < min_distance)
        {
            min_distance = dist;
        }
    }

    if (min_distance < 20)
    {
        return 1.0; // Penalise heavily if the vehicle is too close
    }

    double diff = (REGION_OF_INTEREST_METERS - min_distance);
    return logistic(diff / REGION_OF_INTEREST_METERS);
}

/**
 * @brief Penalises trajectories that does not attempt to go near the MAX_SPEED limit. (i.e. the slower trajectories)
 */
double CostEvaluator::efficient_speed_cost_function(Trajectory const &trajectory,
                                                    const State state) const
{

    auto speed = trajectory.get_average_speed();

    double diff = MAX_SPEED - speed;

    return logistic(diff / MAX_SPEED);
}

/**
 * @brief Penalises trajectories that chooses the lane which has the slow moving vehicles.
 * Note: If the vehicles are outside the Region of Interest then dont penalise.
 * 
 */
double CostEvaluator::efficient_lane_speed_cost_function(Trajectory const &trajectory,
                                                         const State state) const
{
    if (state.d_state == LateralState::CHANGE_LANE_LEFT || state.d_state == LateralState::CHANGE_LANE_RIGHT)
    {
        return 0; // Don't check during lane change since it might lead to indecisive driving.
    }

    std::vector<Vehicle> vehicles_ahead = m_ego.get_vehicles_in_front(m_others, state.future_lane);
    if (vehicles_ahead.size() == 0)
    {
        return 0.0;
    }

    double speed_avg = 0.0;
    int vehicles_count = 0;

    for (const Vehicle &v : vehicles_ahead)
    {
        double dist = distance(m_ego.car_x, m_ego.car_y, v.car_x, v.car_y);
        if (dist <= REGION_OF_INTEREST_METERS * 1.5) // Define a Region of Interest (ROI)
        {
            speed_avg += v.get_current_speed();
            ++vehicles_count;
        }
    }

    if (vehicles_count == 0)
    {
        return 0.0;
    }

    speed_avg /= (double)vehicles_count;

    // Limit lane changes if the average speed is greater than MAX_SPEED.
    if (speed_avg >= MAX_SPEED)
    {
        return 0.0; // Also makes sure the diff >= 0.0.
    }

    double diff = (MAX_SPEED - speed_avg);
    return logistic(diff / MAX_SPEED);
}

/**
 * @brief Penalises trajectories that does not try to match the speed of the vehicle in front.
 * Note: Our goal is to match the speed of the vehicle ahead within a certain region of interest.
 * But if our car is travelling faster than vehicle ahead we should rather maintain/reduce our speed.
 */
double CostEvaluator::difference_in_speed_to_vehicle_ahead_cost_function(Trajectory const &trajectory,
                                                                         const State state) const
{
    std::vector<Vehicle> vehicles_ahead = m_ego.get_vehicles_in_front(m_others, state.current_lane);
    if (vehicles_ahead.size() == 0)
    {
        return 0.0; // No car ahead so dont penalize.
    }

    double min_distance = REGION_OF_INTEREST_METERS; // Define the Region of Interest
    Vehicle closest_vehicle;
    for (const Vehicle &v : vehicles_ahead)
    {
        // TODO: Is the check necessary? We get vehicles only in the current_lane.
        if (v.car_s > m_ego.car_s) // Limit the vehicles to our lane
        {
            double dist = distance(m_ego.car_x, m_ego.car_y, v.car_x, v.car_y);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }

    if (min_distance >= REGION_OF_INTEREST_METERS)
    {
        return 0.0; // The vehicle is outside ROI so don't penalise
    }

    double ego_speed = trajectory.get_average_speed();
    double closest_vehicle_speed = closest_vehicle.get_current_speed();

    // We don't ego to go faster than the vehicle ahead within ROI since this might lead to a collision.
    if (ego_speed > closest_vehicle_speed)
    {
        return 1.0;
    }

    // Match the speed of the vehicle ahead.
    double diff = (closest_vehicle_speed - ego_speed);

    return logistic(diff / closest_vehicle_speed);
}

/**
 * @brief Penalises trajectories that has acceleration (max. value) beyond the allowed limit (MAX_ACCELERATION).
 */
double CostEvaluator::difference_in_max_acceleration_cost_function(Trajectory const &trajectory,
                                                                   const State state) const
{

    double max_acceleration = trajectory.get_max_accerelation();

    return (abs(max_acceleration) > MAX_ACCELERATION) ? 1 : 0;
}

/**
 * @brief Penalises trajectories that has acceleration (avg. value) beyond the allowed limit (MAX_ACCELERATION).
 * Note: Zero acceleration trajectories are sometimes seen as good option because of low cost. 
 * So penalise zero acceleration too.
 */
double CostEvaluator::difference_in_average_acceleration_cost_function(Trajectory const &trajectory,
                                                                       const State state) const
{

    double average_acceleration = trajectory.get_average_acceleration()[0];

    if (average_acceleration == 0) // Penalise also zero acceleration
    {
        return 1;
    }

    return logistic(average_acceleration / MAX_ACCELERATION);
}

/**
 * @brief Penalises trajectories that has a collision path to the other vehicles in front.
 * Note: Applicable for collision only within the Region of Interest
 */
double CostEvaluator::frontal_collision_cost_function(const Trajectory &trajectory, const State state) const
{

    double min_distance = REGION_OF_INTEREST_METERS; // Define a Region of Interest (ROI)
    Vehicle closest_vehicle;                         // The closest vehicle in front to our ego car within the region of interest
    for (const Vehicle &v : m_others)
    {
        if ((v.car_lane == state.current_lane || v.car_lane == state.future_lane) && v.car_s >= m_ego.car_s)
        {
            double dist = distance(m_ego.car_x, m_ego.car_y, v.car_x, v.car_y);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }

    if (min_distance >= REGION_OF_INTEREST_METERS)
    {
        return 0.0; // No need to penalize if the other vehicle is outside the ROI
    }

    CollisionInfo collision_info = trajectory.check_collision(closest_vehicle, SIMULATOR_DELTA_T, FRONTAL_COLLISION_THRESHOLD_METERS);
    if (!collision_info.collision)
    {
        return 0.0; // No cost for collision is not found
    }

    return 1.0;
}

/**
 * @brief Penalises trajectories that has a collision path to the other vehicles in rear and lateral sides.
 * Note: Applicable only when changing lanes.
 */
double CostEvaluator::rear_and_lateral_collision_cost_function(Trajectory const &trajectory, const State state) const
{
    if (!(state.d_state == LateralState::PREPARE_CHANGE_LANE_LEFT || state.d_state == LateralState::PREPARE_CHANGE_LANE_RIGHT))
    {
        return 0.0; // quickly return if not changing lanes
    }

    double min_distance = REGION_OF_INTEREST_METERS; // Define a Region of Interest (ROI)
    Vehicle closest_vehicle;                         // The closest vehicle in rear/lateral to our ego car within the region of interest
    for (const Vehicle &v : m_others)
    {
        if ((v.car_lane == state.current_lane || v.car_lane == state.future_lane) && v.car_s <= m_ego.car_s)
        {
            double dist = distance(m_ego.car_x, m_ego.car_y, v.car_x, v.car_y);
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_vehicle = v;
            }
        }
    }

    if (min_distance >= REGION_OF_INTEREST_METERS)
    {

        return 0.0; // No need to penalize if the other vehicle is outside the ROI
    }

    CollisionInfo collision_info = trajectory.check_collision(closest_vehicle, SIMULATOR_DELTA_T, REAR_COLLISION_THRESHOLD_METERS);

    if (collision_info.collision && (state.d_state == LateralState::PREPARE_CHANGE_LANE_LEFT || state.d_state == LateralState::PREPARE_CHANGE_LANE_RIGHT))
    {
        return 1.0;
    }

    return 0.0;
}

// Cost functions that could be useful

// /**
//  * @brief Penalises trajectories that has maximum distance to the goal at the end of the trajectory.
//  */
// double CostEvaluator::distance_to_goal_cost_function(Trajectory const &trajectory, const State state)
// {
//     int traj_size = trajectory.size();

//     double diff = MAX_TRACK_S - trajectory.s_vector[traj_size - 1];

//     return logistic(diff / MAX_TRACK_S);
// }

// /**
//  * @brief Penalises trajectories that has jerk (avg. value) beyond the allowed limit (MAX_JERK).
//  */
// double CostEvaluator::difference_in_average_jerk_cost_function(Trajectory const &trajectory,
//                                                                const State state)
// {

//     double average_jerk = trajectory.get_average_jerk()[0];

//     return logistic(average_jerk / MAX_JERK);
// }

// /**
//  * @brief Penalises trajectories that has jerk (max. value) beyond the allowed limit (MAX_JERK).
//  */
// double CostEvaluator::difference_in_max_jerk_cost_function(Trajectory const &trajectory,
//                                                            const State state)
// {

//     double max_jerk = trajectory.get_max_jerk();

//     return (abs(max_jerk) > MAX_JERK) ? 1 : 0;
// }
