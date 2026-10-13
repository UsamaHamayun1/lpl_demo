# The authority envelope: every permission, every time (never deltas).
# Published by the LPL authority node on change and as a heartbeat.
std_msgs/Header header
uint32 revision          # increases whenever any permission changes
float32 lease_s          # consumers treat a state older than this as "nothing authorized"
string mode              # "lpl", "permissive", "global_revoke", "MOCK", ...
string rules_id          # which rule set produced this
PermissionState[] permissions
string cause             # what triggered the latest change, e.g. "E3 INSUFFICIENT (OCCLUDED)"
