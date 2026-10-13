# Whether the human is needed, and for what.
uint8 NONE=0
uint8 ADVISORY=1                  # robot continues; a better option needs a permission
uint8 HUMAN_JUDGMENT_REQUIRED=2   # no adequate permitted option remains
uint8 SYSTEM_FAULT=3              # something is broken (stale data, enforcement leak, ...), not a judgment call

uint8 KIND_NONE=0
uint8 KIND_BETTER_OPTION_NEEDS_PERMISSION=1
uint8 KIND_AUTHORITY_EXHAUSTED=2  # a possible option is blocked only by grantable permissions
uint8 KIND_NO_FEASIBLE=3          # nothing reachable is physically possible
uint8 KIND_LPL_STALE=4
uint8 KIND_EVIDENCE_STALE=5
uint8 KIND_ENFORCEMENT_LEAK=6
uint8 KIND_EGRESS_OVERRUN=7
uint8 KIND_EXECUTION_FAILED=8     # the selected option keeps failing in execution although it is possible

std_msgs/Header header
uint8 level
uint8 kind
string headline              # e.g. "HUMAN JUDGMENT REQUIRED"
string[] option_lines        # one line per option explaining its status
string[] asks                # permission ids the human could grant to help, e.g. ["P3"]
string advised_option        # option that would become available
float32 eta_gain_s           # time saved by granting, for ADVISORY
builtin_interfaces/Time since
