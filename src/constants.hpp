#define MAX_SPEED (48 * 0.44704) // 47 mph in m/s

#define SIMULATOR_DELTA_T 0.02 // each point is visited in 0.02 s in the simulator
#define TIME_INTERVAL 2.0 // The trajectory is generated for the given number of seconds

#define MAX_ACCELERATION 10.0 // Rate of change of average speed over 0.2 seconds
#define MAX_JERK 10.0 // Average acceleration over 1 second intervals

#define PATH_LENGTH 100  // Number of points that make up a single trajectory output to the simulator
#define RETAIN_PATH_LENGTH 45 // The number of points to be used from the previous/old trajectory. This results in a smoother trajectory

#define MAX_TRACK_S 6945.554

#define NUM_LANES 3 
#define LANE_WIDTH  4.0 // Width of the lane which is 4m


#define  REGION_OF_INTEREST_METERS 50.0 // Constant that defines the Region of Interest
#define  FRONTAL_COLLISION_THRESHOLD_METERS  15.0 // Threshold for collision in forward direction
#define  REAR_COLLISION_THRESHOLD_METERS  15.0 // Threshold for collision in rear direction

// Weights for the different cost functions

# define SPEED_LIMIT_WEIGHT 500.0
# define EFFICIENT_SPEED_WEIGHT 70.0

# define MAX_ACCELERATION_WEIGHT 100.0
# define TOTAL_ACCELERATION_WEIGHT 10.0


# define EFFICIENT_LANE_SPEED_WEIGHT 200.0
# define DISTANCE_TO_VEHICLE_AHEAD_WEIGHT 120.0
# define SPEED_DIFFERENCE_TO_VEHICLE_AHEAD_WEIGHT 70.0

# define FRONTAL_COLLISION_WEIGHT 500.0
# define REAR_COLLISION_WEIGHT 300.0

# define STAYS_ON_ROAD_WEIGHT 300.0
# define DISTANCE_TO_CENTER_WEIGHT 100.0


// Not used
# define DISTANCE_TO_GOAL_WEIGHT 70.0
# define MAX_JERK_WEIGHT 100.0
# define TOTAL_JERK_WEIGHT 10.0
