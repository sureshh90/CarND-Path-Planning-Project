#include "trajectory.hpp"

Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::add_point(double x, double y,
                     double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
                     double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
                     double yaw)
{
    this->x_vector.push_back(x);
    this->y_vector.push_back(y);

    this->s_vector.push_back(s);
    this->s_dot_vector.push_back(s_dot);
    this->s_dot_dot_vector.push_back(s_dot_dot);
    this->s_jerk_vector.push_back(s_dot_dot_dot);

    this->d_vector.push_back(d);
    this->d_dot_vector.push_back(d_dot);
    this->d_dot_dot_vector.push_back(d_dot_dot);
    this->d_jerk_vector.push_back(d_dot_dot_dot);

    this->yaw_vector.push_back(yaw);
}

int Trajectory::size() const
{
    return this->x_vector.size();
}

double Trajectory::get_average_speed() const
{

    int points_count = this->size();

    if (points_count < 2)
    {
        return 0.0;
    }

    double velocity_sum = std::accumulate(s_dot_vector.begin(), s_dot_vector.end(), 0.0);

    return velocity_sum / points_count;
}

double Trajectory::get_max_speed() const
{
    return *(std::max_element(s_dot_vector.begin(), s_dot_vector.end()));
}

double Trajectory::get_max_accerelation() const
{
    return *(std::max_element(s_dot_dot_vector.begin(), s_dot_dot_vector.end()));
}

double Trajectory::get_max_jerk() const
{
    return *(std::max_element(s_jerk_vector.begin(), s_jerk_vector.end()));
}

std::vector<double> Trajectory::get_average_acceleration() const
{
    int points_count = this->size();
    if (points_count < 2)
    {
        return {0.0, 0.0};
    }

    double s_acceleration_sum = std::accumulate(s_dot_dot_vector.begin(), s_dot_dot_vector.end(), 0.0, [](double accumulator, double const &s_dot_dot)
                                                { return accumulator + abs(s_dot_dot) * SIMULATOR_DELTA_T; });

    double d_acceleration_sum = std::accumulate(d_dot_dot_vector.begin(), d_dot_dot_vector.end(), 0.0, [](double accumulator, double const &d_dot_dot)
                                                { return accumulator + abs(d_dot_dot) * SIMULATOR_DELTA_T; });

    return {s_acceleration_sum / (points_count * SIMULATOR_DELTA_T), d_acceleration_sum / (points_count * SIMULATOR_DELTA_T)};
}

std::vector<double> Trajectory::get_average_jerk() const
{
    int points_count = this->size();
    if (points_count < 2)
    {
        return {0.0, 0.0};
    }

    double s_jerk_sum = std::accumulate(s_jerk_vector.begin(), s_jerk_vector.end(), 0.0, [](double accumulator, double const &s_dot_dot_dot)
                                        { return accumulator + abs(s_dot_dot_dot) * SIMULATOR_DELTA_T; });

    double d_jerk_sum = std::accumulate(d_jerk_vector.begin(), d_jerk_vector.end(), 0.0, [](double accumulator, double const &d_dot_dot_dot)
                                        { return accumulator + abs(d_dot_dot_dot) * SIMULATOR_DELTA_T; });

    return {s_jerk_sum / (points_count * SIMULATOR_DELTA_T), d_jerk_sum / (points_count * SIMULATOR_DELTA_T)};
}

void Trajectory::erase_n_at_start(int n)
{
    this->x_vector.erase(this->x_vector.begin(), this->x_vector.begin() + n);
    this->y_vector.erase(this->y_vector.begin(), this->y_vector.begin() + n);

    this->s_vector.erase(this->s_vector.begin(), this->s_vector.begin() + n);
    this->s_dot_vector.erase(this->s_dot_vector.begin(), this->s_dot_vector.begin() + n);
    this->s_dot_dot_vector.erase(this->s_dot_dot_vector.begin(), this->s_dot_dot_vector.begin() + n);
    this->s_jerk_vector.erase(this->s_jerk_vector.begin(), this->s_jerk_vector.begin() + n);

    this->d_vector.erase(this->d_vector.begin(), this->d_vector.begin() + n);
    this->d_dot_vector.erase(this->d_dot_vector.begin(), this->d_dot_vector.begin() + n);
    this->d_dot_dot_vector.erase(this->d_dot_dot_vector.begin(), this->d_dot_dot_vector.begin() + n);
    this->d_jerk_vector.erase(this->d_jerk_vector.begin(), this->d_jerk_vector.begin() + n);

    this->yaw_vector.erase(this->yaw_vector.begin(), this->yaw_vector.begin() + n);
}

Trajectory Trajectory::copy_n_upto_index(int up_to_index) const
{
    Trajectory copy = Trajectory();
    for (int i = 0; i < this->size() && i <= up_to_index; ++i)
    {
        copy.add_point(this->x_vector[i], this->y_vector[i],
                 this->s_vector[i], this->s_dot_vector[i], this->s_dot_dot_vector[i], this->s_jerk_vector[i],
                 this->d_vector[i], this->d_dot_vector[i], this->d_dot_dot_vector[i], this->d_jerk_vector[i],
                 this->yaw_vector[i]);
    }

    return copy;
}


CollisionInfo Trajectory::check_collision(const Vehicle &vehicle, double timestep, double threshold) const
{
    Map &map = Map::get_map_instance();

    for (int i = 0; i < this->size(); ++i)
    {
        double ref_x = this->x_vector[i];
        double ref_y = this->y_vector[i];

        double v_predicted_x = vehicle.car_x + vehicle.car_vx * timestep * i;
        double v_predicted_y = vehicle.car_y + vehicle.car_vy * timestep * i;

        vector<double> frenet = map.get_frenet(v_predicted_x, v_predicted_y, vehicle.car_yaw);

        double dist = distance(ref_x, ref_y, v_predicted_x, v_predicted_y);

        if (dist < threshold)
        {
            // std::cout << "Debug: Collision timestep = " << i << " with parameters "<< ref_x << " " << ref_y << " " << v_predicted_x << " "  << v_predicted_y << " " << std::endl;
            return CollisionInfo(vehicle, true, ref_x, ref_y, (double)i);
        }
    }
    return CollisionInfo(vehicle, false, 0.0, 0.0, 0.0);
}
