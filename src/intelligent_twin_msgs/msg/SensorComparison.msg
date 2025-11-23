# Sensor comparison between real robot and digital twin
# Used for anomaly detection feature extraction

std_msgs/Header header

# Position differences
float32 position_diff_x          # X deviation in meters
float32 position_diff_y          # Y deviation in meters
float32 position_diff_total      # Euclidean distance

# Orientation difference
float32 orientation_diff         # Theta deviation in radians

# Velocity differences
float32 linear_vel_diff          # Linear velocity difference (m/s)
float32 angular_vel_diff         # Angular velocity difference (rad/s)

# Laser scan comparison
float32 scan_diff_mean           # Average scan difference
float32 scan_diff_max            # Maximum scan difference
float32 scan_diff_variance       # Variance in differences
uint32 scan_mismatches           # Number of readings significantly different

# Command synchronization
bool commands_match              # Did both receive same command?
float32 sync_delay               # Time between command receipts
