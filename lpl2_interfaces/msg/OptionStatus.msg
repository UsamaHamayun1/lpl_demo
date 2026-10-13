# How the planner sees one way of continuing the mission.

uint8 ROUTE=0
uint8 HOLD=1
uint8 EGRESS=2

string id                # "A", "B", "H"
string name              # "Pass through Lane 1"
uint8 kind
string[] requires        # permissions the remaining part of this option needs
string[] missing         # of those, the ones not currently usable
bool reachable           # the robot can still start or continue it (false after a commit point)
bool possible            # physically feasible right now
string possible_reason   # why not, e.g. "bypass door blocked (lidar)"
bool evidenced           # every required permission is supported by evidence
bool authorized          # every required permission is usable (evidence or human grant)
bool adequate            # good enough to pursue the mission
string inadequate_reason
bool selected
float32 eta_s            # estimated time to the goal, < 0 if unknown
float32 remaining_m
geometry_msgs/Point[] path   # map frame, for drawing
