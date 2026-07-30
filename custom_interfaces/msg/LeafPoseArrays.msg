# Poses1 includes all the midpoints and rotations for all detected targets.
# e.g Poses2 includes same midpoints but with new rotations for all detected targets (could be
# 45 degrees CC rotation around the normal vector of the targets calculated in Poses1)

std_msgs/Header header
geometry_msgs/Pose[] poses1
geometry_msgs/Pose[] poses2
geometry_msgs/Pose[] poses3
geometry_msgs/Pose[] poses4
geometry_msgs/Pose[] poses5

# Reachability-aware extension. Each flat pose references one of the point
# metadata rows below and records its approach tilt and romu4o roll.
geometry_msgs/Pose[] candidate_poses
int32[] candidate_point_indices
float32[] candidate_tilt_degrees
float32[] candidate_roll_degrees

# Candidate metadata.  Every entry describes the position shared by poses1..5
# at the same array index.  The five pose arrays rotate the RG2 about the
# estimated leaf normal, preserving the original romu4o message convention.
int32[] leaf_ids
float32[] scores
float32[] geometric_scores
float32[] confidences
int32[] view_counts
float32[] edge_margins
float32[] local_leaf_widths
float32[] edge_margin_ratios
float32[] approach_clearances
float32[] longitudinal_ratios
float32[] projection_distances
string[] collision_leaf_ids
string selection_method
