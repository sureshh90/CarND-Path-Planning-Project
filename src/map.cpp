#include <vector>
#include "map.hpp"


Map &Map::get_map_instance()
{
    static Map instance;
    return instance;
}

void Map::add_waypoint(double x, double y, float s, float d_x, float d_y)
{
    // Adds the map waypoints
    m_map_waypoints_x.push_back(x);
    m_map_waypoints_y.push_back(y);
    m_map_waypoints_s.push_back(s);
    m_map_waypoints_dx.push_back(d_x);
    m_map_waypoints_dy.push_back(d_y);
}

void Map::build_splines()
{
    m_spline_s_vs_x.set_points(m_map_waypoints_s, m_map_waypoints_x);
    m_spline_s_vs_y.set_points(m_map_waypoints_s, m_map_waypoints_y);
    m_spline_s_vs_dx.set_points(m_map_waypoints_s, m_map_waypoints_dx);
    m_spline_s_vs_dy.set_points(m_map_waypoints_s, m_map_waypoints_dy);
}

// Calculate closest waypoint to current x, y position
int Map::get_closest_way_point(double x, double y)
{
    double closestLen = 100000; // large number
    int closestWaypoint = 0;

    for (int i = 0; i < m_map_waypoints_x.size(); ++i)
    {
        double map_x = m_map_waypoints_x[i];
        double map_y = m_map_waypoints_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int Map::get_next_way_point(double x, double y, double theta)
{
    int closestWaypoint = this->get_closest_way_point(x, y);

    double map_x = m_map_waypoints_x[closestWaypoint];
    double map_y = m_map_waypoints_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2)
    {
        ++closestWaypoint;
        if (closestWaypoint == m_map_waypoints_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Map::get_frenet(double x, double y, double theta)
{
    int next_wp = this->get_next_way_point(x, y, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = m_map_waypoints_x.size() - 1;
    }

    double n_x = m_map_waypoints_x[next_wp] - m_map_waypoints_x[prev_wp];
    double n_y = m_map_waypoints_y[next_wp] - m_map_waypoints_y[prev_wp];
    double x_x = x - m_map_waypoints_x[prev_wp];
    double x_y = y - m_map_waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - m_map_waypoints_x[prev_wp];
    double center_y = 2000 - m_map_waypoints_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance(m_map_waypoints_x[i], m_map_waypoints_y[i], m_map_waypoints_x[i + 1], m_map_waypoints_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Map::get_XY(double s, double d)
{
    int prev_wp = -1;

    while (s > m_map_waypoints_s[prev_wp + 1] && (prev_wp < (int)(m_map_waypoints_s.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % m_map_waypoints_x.size();

    double heading =
        atan2((m_map_waypoints_y[wp2] - m_map_waypoints_y[prev_wp]), (m_map_waypoints_x[wp2] - m_map_waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - m_map_waypoints_s[prev_wp]);

    double seg_x = m_map_waypoints_x[prev_wp] + seg_s * cos(heading);
    double seg_y = m_map_waypoints_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

vector<double> Map::get_spline_XY(double s, double d)
{
    // build_splines():
    s = fmod(s, MAX_TRACK_S); // To take wrap around of lanes into account
    double x = m_spline_s_vs_x(s) + d * m_spline_s_vs_dx(s);
    double y = m_spline_s_vs_y(s) + d * m_spline_s_vs_dy(s);

    return {x, y};
}
