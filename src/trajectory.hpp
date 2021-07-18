#ifndef TRAJECTORY_H
#define TRAJECTORY_H

struct CollisionInfo
{
    const Vehicle &vehicle;
    const bool collision;
    const double collision_point_x;
    const double collision_point_y;
    const double collision_timestep;

    CollisionInfo(const Vehicle &vehicle, const bool collision,
                  const double collision_point_x, const double collision_point_y, const double collision_timestep)
        : vehicle(vehicle), collision(collision), collision_point_x(collision_point_x), collision_point_y(collision_point_y), collision_timestep(collision_timestep){};

    ~CollisionInfo(){};
};

class Trajectory
{
public:
    std::vector<double> x_vector;
    std::vector<double> y_vector;

    std::vector<double> s_vector;
    std::vector<double> s_dot_vector;
    std::vector<double> s_dot_dot_vector;
    std::vector<double> s_jerk_vector;

    std::vector<double> d_vector;
    std::vector<double> d_dot_vector;
    std::vector<double> d_dot_dot_vector;
    std::vector<double> d_jerk_vector;

    std::vector<double> yaw_vector;

    Trajectory();

    virtual ~Trajectory();

    void add_point(double x, double y,
             double s, double s_dot, double s_dot_dot, double s_dot_dot_dot,
             double d, double d_dot, double d_dot_dot, double d_dot_dot_dot,
             double yaw);

    int size() const;

    double get_average_speed() const;

    double get_max_speed() const;

    double get_max_accerelation() const;

    double get_max_jerk() const;

    vector<double> get_average_acceleration() const;

    vector<double> get_average_jerk() const;

    void erase_n_at_start(int n);

    Trajectory copy_n_upto_index(int up_to_index) const;

    CollisionInfo check_collision(const Vehicle &vehicle, double timestep, double threshold) const;
};

#endif