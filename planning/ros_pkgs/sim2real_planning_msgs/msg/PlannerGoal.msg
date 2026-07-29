# A gateway-assigned goal request. All IDs are scoped to session_id.
std_msgs/Header header
string session_id
uint64 goal_id

uint8 PLAN=0
uint8 CANCEL=1
uint8 action

geometry_msgs/PoseStamped goal
# False means the backend may choose path-aligned yaw. In that case goal's
# orientation is intentionally the zero quaternion.
bool constrain_yaw
