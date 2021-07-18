#include "vehicle.hpp"

Vehicle::Vehicle(double id)
{
    car_id = id;
    car_x = 0;
    car_y = 0;
    car_s = 0;
    car_d = 0;
    car_yaw = 0;
    car_speed = 0;
    car_vx = 0;
    car_vy = 0;
}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d)
{
    this->car_id = id;
    this->car_x = x;
    this->car_y = y;
    this->car_vx = vx;
    this->car_vy = vy;
    this->car_s = s;
    this->car_d = d;;
    this->car_yaw = get_yaw_in_radians(vx, vy);
    this->car_lane = get_lane_index(this->car_d);
}

void Vehicle::update_localization(double x, double y, double s, double d,
                                  double yaw, double speed)
{
    // Main car's localization Data
    this->car_x = x;
    this->car_y = y;
    this->car_vx = cos(deg2rad(yaw));
    this->car_vy = sin(deg2rad(yaw));
    this->car_s = s;
    this->car_d = d;
    this->car_yaw = deg2rad(yaw);      // our yaw is in radians
    this->car_speed = speed * 0.44704; // our speed is always in meters per second
}

double Vehicle::get_current_speed() const
{
    return sqrt(this->car_vx * this->car_vx + this->car_vy * this->car_vy);
}

std::vector<Vehicle> Vehicle::get_vehicles_in_front(const std::vector<Vehicle> &others, int lane) const
{
    std::vector<Vehicle> vehicles_in_front;

    std::for_each(others.begin(), others.end(), [&](Vehicle const &v)
                  {
                      if (v.car_lane == lane && v.car_s >= this->car_s)
                      {
                          vehicles_in_front.push_back(v);
                      }
                      return;
                  });

    return vehicles_in_front;
}

std::vector<Vehicle> Vehicle::get_vehicles_in_rear(const std::vector<Vehicle> &others, int lane) const
{
    std::vector<Vehicle> vehicles_in_rear;

    std::for_each(others.begin(), others.end(), [&](Vehicle const &v)
                  {
                      if (v.car_lane == lane && v.car_s < this->car_s)
                      {
                          vehicles_in_rear.push_back(v);
                      }
                      return;
                  });

    return vehicles_in_rear;
}