# Localization state (g1d_localization localization_monitor), 2 Hz.
std_msgs/Header header

string mode            # map | odom
bool localized         # odom mode: always true (map -> odom is identity)
string[] reasons       # no_registration | not_converged | low_inliers | registration_failing |
                       # registration_slow (still localized) | registration_stale | no_map
float32 inlier_ratio   # last registration: inliers / source points
float32 error          # last registration error (small_gicp)
float32 last_ok_age    # seconds since the last good registration, -1 if never
float32 correction     # change of map -> odom translation at the last registration (m)
geometry_msgs/Pose2D pose   # base_link in map
