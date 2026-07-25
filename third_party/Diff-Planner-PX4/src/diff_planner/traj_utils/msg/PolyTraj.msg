int16 drone_id
int32 traj_id
time start_time

uint8 order
float32[] coef_x
float32[] coef_y
float32[] coef_z
float32[] duration

# Final pose metadata for the global goal associated with this local trajectory.
# The trajectory server uses it to keep path-aligned yaw during transit and to
# track the requested goal yaw near/at the destination.
bool has_goal_yaw
float64 goal_yaw
float64[3] goal_position

# Exact source goal request for correlating a planned trajectory with an RViz
# click. Zero for preset waypoint missions that have no PoseStamped request.
time goal_stamp

# True only for a successfully generated normal flight trajectory. Emergency
# stop/hold trajectories must never be used as evidence for automatic arming.
bool armable
