# Navigation pose lock of the G1-D upper body (the head lidar must not move relative to the chassis).
std_msgs/Header header

bool locked
# reasons: stale_lift | stale_lowstate | stale_waist_pitch | waist_yaw | waist_pitch | lift_moving | lift_changed | arms
string[] reasons
float32 lift_height  # m (URDF total lift travel)
float32 lift_speed   # m/s
float32 waist_pitch  # rad
float32 waist_yaw    # rad
