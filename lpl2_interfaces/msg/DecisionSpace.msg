# The planner's view of the available decision space, published every planning cycle.
std_msgs/Header header
uint32 cycle
uint32 authority_revision    # AuthorityState.revision this decision is based on
string mission_id
geometry_msgs/Point goal
OptionStatus[] options
string selected_id
string selection_reason
string[] faults              # invariant violations, e.g. "ENFORCEMENT_LEAK"
