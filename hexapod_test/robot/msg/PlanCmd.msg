std_msgs/Header header
# robot state now
geometry_msgs/Pose robot_pose_now
geometry_msgs/Twist robot_twist_now
# planned robot state and foot state
geometry_msgs/Pose robot_pose_next
geometry_msgs/Twist robot_twist_next
geometry_msgs/Point[] foot_pos_next
bool[] foot_contact_next
float64 step_time
float64 foot_lift_height
