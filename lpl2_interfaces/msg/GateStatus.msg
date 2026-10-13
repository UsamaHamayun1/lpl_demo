# The execution gate: the only writer of /cmd_vel.
uint8 PASS=0
uint8 BLOCK_PERMISSION=1     # command would use a region/permission that is not usable
uint8 BLOCK_LEASE=2          # authority state too old
uint8 WATCHDOG=3             # autonomy command stream went silent
uint8 ESTOP=4
uint8 MANUAL=5               # human teleop takeover
uint8 EGRESS=6               # leaving a revoked region, bounded

std_msgs/Header header
uint8 state
string reason
uint32 blocks_total
float32 linear               # command actually sent to the robot
float32 angular
