# What the executor is doing with the selected option.
uint8 IDLE=0
uint8 EXECUTING=1
uint8 HOLDING=2
uint8 EGRESS=3
uint8 SUCCEEDED=4
uint8 FAILED=5
uint8 REFUSED=6          # dispatch check refused an option whose permissions are not usable

std_msgs/Header header
string option_id
uint8 state
uint16 nav2_error_code
string detail
float32 progress_m
float32 remaining_m
