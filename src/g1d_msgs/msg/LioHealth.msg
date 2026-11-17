# Health of the LIO odometry (g1d_lio_bridge), published at 5 Hz.
std_msgs/Header header

bool ok
# errors: no_lio | lio_timeout | jump | z_drift | tilt | wheel_inconsistent | no_tf
string[] errors
float32 age          # seconds since the last FAST_LIO pose
float32 z            # base_link height in odom (m), ~0 on flat floors
float32 roll         # rad
float32 pitch        # rad
float32 wheel_dpos   # LIO vs wheel displacement difference over the window (m)
float32 wheel_dyaw   # (rad)
