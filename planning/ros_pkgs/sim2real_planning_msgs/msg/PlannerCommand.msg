# One setpoint emitted by a planner backend. Only the gateway may convert this
# into the public /command/trajectory controller stream.
std_msgs/Header header
string session_id
string backend_id
uint64 goal_id
uint64 trajectory_id

uint8 NORMAL=0
uint8 HOLD=1
uint8 BRAKE=2
uint8 mode

trajectory_msgs/MultiDOFJointTrajectoryPoint point
