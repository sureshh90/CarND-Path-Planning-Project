#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"

#include "json.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

#include "map.hpp"
#include "vehicle.hpp"
#include "vehicle.cpp"
#include "vehiclebehaviour.cpp"
#include "trajectorygenerator.cpp"
#include "trajectory.hpp"
#include "trajectory.cpp"
#include "map.cpp"
#include "statemachine.hpp"
#include "cost.hpp"
#include "cost.cpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  // Declare objects for different classes
  Map &map = Map::get_map_instance();
  Vehicle ego(-1);
  std::vector<Vehicle> other_vehicles;
  VehicleBehaviour egoBehaviour(ego, other_vehicles);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map.add_waypoint(x, y, s, d_x, d_y);
  }

  //TODO: Check if this can be done inside Map class
  map.build_splines();

  h.onMessage([&map, &ego, &egoBehaviour, &other_vehicles](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                                             size_t length, uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Update the car data
                      ego.update_localization(car_x, car_y, fmod(car_s, MAX_TRACK_S), car_d, car_yaw, car_speed);

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      for (auto v_data : sensor_fusion)
                      {
                        Vehicle v = Vehicle(v_data[0], v_data[1], v_data[2], v_data[3], v_data[4], v_data[5], v_data[6]);
                        other_vehicles.push_back(v);
                      }

                      json msgJson;

                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      // Start: Logic for trajectory generation
                      int previous_path_size = previous_path_x.size();

                      Trajectory new_trajectory = egoBehaviour.generate_optimal_trajectory(previous_path_size);
                      // std::cout << "Debug: Size of new trajectory " << new_trajectory.size() << std::endl;

                      for (int i = 0; i < new_trajectory.x_vector.size(); ++i)
                      {
                        next_x_vals.push_back(new_trajectory.x_vector[i]);
                        next_y_vals.push_back(new_trajectory.y_vector[i]);
                      }

                      egoBehaviour.update_current_trajectory(new_trajectory);

                      // clean other_vehicles vector
                      other_vehicles.erase(other_vehicles.begin(), other_vehicles.end());

                      // End: Logic for trajectory generation

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                } // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}