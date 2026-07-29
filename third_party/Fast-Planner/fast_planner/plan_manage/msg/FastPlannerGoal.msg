uint8 PLAN=0
uint8 CANCEL=1

std_msgs/Header header
uint64 goal_id
uint8 action
geometry_msgs/PoseStamped goal
bool constrain_yaw
