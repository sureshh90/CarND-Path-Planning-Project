#ifndef MAP_H
#define MAP_H
#include "spline.h"
#include <vector>

class Map
{

    std::vector<double> m_map_waypoints_x;
    std::vector<double> m_map_waypoints_y;
    std::vector<double> m_map_waypoints_s;
    std::vector<double> m_map_waypoints_dx;
    std::vector<double> m_map_waypoints_dy;

    tk::spline m_spline_s_vs_x;
    tk::spline m_spline_s_vs_y;
    tk::spline m_spline_s_vs_dx;
    tk::spline m_spline_s_vs_dy;

    // C++ 11 technique for singleton
    Map() {}
    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;
    Map(Map &&) = delete;
    Map &operator=(Map &&) = delete;

    int get_closest_way_point(double x, double y);
    int get_next_way_point(double x, double y, double theta);

public:
    static Map &get_map_instance();

    void build_splines();

    void add_waypoint(double x, double y, float s, float d_x, float d_y);

    std::vector<double> get_frenet(double x, double y, double theta);

    std::vector<double> get_XY(double s, double d);

    std::vector<double> get_spline_XY(double s, double d);
};

#endif