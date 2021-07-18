#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
public:
  double car_id;
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  double car_vx;
  double car_vy;
  double car_lane;

  Vehicle(){};
  Vehicle(double id);
  Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

  void update_localization(double car_x, double car_y, double car_s, double car_d,
                           double car_yaw, double car_speed);

  double get_current_speed() const;

  std::vector<Vehicle> get_vehicles_in_front(const std::vector<Vehicle> &others, int lane) const;

  std::vector<Vehicle> get_vehicles_in_rear(const std::vector<Vehicle> &others, int lane) const;
};

#endif