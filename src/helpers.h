#ifndef HELPERS_H
#define HELPERS_H

#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include "constants.hpp"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


int get_lane_index(double d)
{
  //return floor(d / LANE_WIDTH);
  int lane = 10; // Initialize a lane id outside the range

  if (d >= 0.0 && d <= 4.0)
    lane = 0;
  else if (d > 4.0 && d <= 8.0)
    lane = 1;
  else if (d > 8.0  && d <= 12.0)
    lane = 2;

  return lane;
}

bool is_lane_valid(int lane)
{
  return lane >= 0 && lane < NUM_LANES;
}

double get_lane_center_frenet(int lane)
{
  return 2.0 + 4.0 * lane;
}

/**
  @brief A function that returns a value between 0 and 1 for x in the 
  range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

  Useful for cost functions.
  */
double logistic(double x)
{

  return 2.0 / (double)(1.0 + exp(-abs(x))) - 1.0;
  // return 1.0 - exp(-abs(x));
}

/**
 * @brief Takes the coefficients of a polynomial and creates a function of
 * time from them.
 *
 * @param coefficients - the coeffients that needs to be converted to an
 * equation
 *
 * Example: A coefficients array of length 6, each value corresponding to
 * a coefficent in the polynomial: 
 * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * * t**3 + a_4 * t**4 + a_5 * t**5
 *
 */
std::function<double(double)> to_equation(vector<double> coefficients)
{

  return [coefficients](double t)
  {
    double total = 0.0;
    int idx = 0;
    for (auto &elem : coefficients)
    {
      total += elem * std::pow(t, idx);
      idx++;
    }
    return total;
  }; // end of lambda function

} // end of to_equation function

/**
 * @brief Calculates the derivative of a polynomial and returns
 * the corresponding coefficients.
 */
std::vector<double> differentiate(std::vector<double> old_coefficients)
{
  std::vector<double> new_coefficients;
  int idx = 0;

  std::transform(old_coefficients.begin() + 1, old_coefficients.end(), std::back_inserter(new_coefficients),
                 [&idx](double const old_coefficient)
                 {
                   idx++;
                   return old_coefficient * idx;
                 });

  return new_coefficients;
}

/**
 * @brief Calculates the derivatives of a polynomial upto to N degree
 */
std::vector<std::function<double(double)>> get_function_and_its_derivatives(std::vector<double> coefficients, int N = 3)
{
  std::vector<std::function<double(double)>> derivative_functions;
  derivative_functions.push_back(to_equation(coefficients));

  std::vector<double> differentiated_coefficients = coefficients;

  for (int i = 0; i < N; i++)
  {
    differentiated_coefficients = differentiate(differentiated_coefficients);
    derivative_functions.push_back(to_equation(differentiated_coefficients));
  }

  return derivative_functions;
}

/**
 * 
 * @brief The function returns the angle theta in radians, based on the supplied velocity components
 */
double get_yaw_in_radians(double vx, double vy)
{
  return atan2(vy, vx);
}

#endif // HELPERS_H