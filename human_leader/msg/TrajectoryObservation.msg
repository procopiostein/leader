uint8 FIRST = 1 #type should be 1 if this observation correspons to an starting point
uint8 LAST  = 2 #2 if it is the final point of a trajectory
Header header
uint32 object_id
uint8 type
geometry_msgs/Pose2D pose
geometry_msgs/Twist velocity
