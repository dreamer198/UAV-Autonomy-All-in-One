# Runtime capabilities must match the selected planner.plugin.yaml manifest.
std_msgs/Header header
string api_version
string backend_id
string variant

bool simulation
bool real_flight
bool yaw
bool cancel
bool goal_validation
bool rviz

float64 max_velocity
float64 max_acceleration

bool has_fixed_map_bounds
geometry_msgs/Point map_min
geometry_msgs/Point map_max
