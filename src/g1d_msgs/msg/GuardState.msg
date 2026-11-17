# State of g1d_guard (the only writer of /cmd_vel_no_limit), published at 10 Hz.
std_msgs/Header header

float32 v            # command written to the chassis (m/s)
float32 w            # (rad/s)
float32 v_cmd        # last navigation command
float32 w_cmd
float32 v_target     # target after source selection, limits and reflexes (before rate limiting)
float32 w_target

string blocked       # "" | front | rear | rotate | scan_stale | cloud_stale
bool estop
string estop_reason  # topic | bumper_triggered | cliff_detected
bool manual          # remote controller or teleop owns the base
string manual_source # "" | remote | web
string source        # nav | teleop | none
bool publishing      # /cmd_vel_no_limit is being written

float32 front_clear  # swept clearance ahead / behind (m), +inf if nothing in range
float32 rear_clear
bool rotate_ok

float32 cmd_age      # seconds since the last input, -1 if never received
float32 scan_age
float32 cloud_age
