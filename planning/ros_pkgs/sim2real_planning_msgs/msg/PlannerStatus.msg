# Lifecycle and correlation state for one planner backend.
std_msgs/Header header
string session_id
string backend_id
uint64 goal_id
uint64 trajectory_id
geometry_msgs/PoseStamped active_goal

uint8 STARTING=0
uint8 READY=1
uint8 PLANNING=2
uint8 ACTIVE=3
uint8 HOLDING=4
uint8 REACHED=5
uint8 FAULT=6
uint8 state

bool odom_ready
bool map_ready
bool armable
string reason
