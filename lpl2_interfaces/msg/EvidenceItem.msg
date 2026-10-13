# One piece of evidence about the world, produced by the evidence layer
# (scripted for the demo; Quang's perception later). Evidence never names permissions:
# the LPL rules map evidence to permissions.

# Zero is UNKNOWN on purpose: a default-constructed item must never read as ADEQUATE.
uint8 UNKNOWN=0          # no data, or older than ttl_s (consumers treat stale items as UNKNOWN)
uint8 ADEQUATE=1
uint8 INSUFFICIENT=2
uint8 CONTRADICTED=3     # evidence actively against the proposition (e.g. oncoming robot seen in the
                         # sector); unlike INSUFFICIENT it also cancels any human grant that relies on it

string id                # stable id, e.g. "E3_lane1_sector_clear"
string proposition       # what is claimed, e.g. "Lane 1 verified clear through the passing sector"
uint8 level
string reason            # short code, e.g. "CLEAR", "OCCLUDED", "BLOCKED"
string detail            # human-readable explanation shown in the UI
float32 confidence       # 0..1
builtin_interfaces/Time observed_at    # sim time (use_sim_time) when the observation was made
float32 ttl_s            # valid for this long after observed_at; 0 = use the consumer's default
string source            # "scripted", "los", "perception", ...
bool scripted            # true if the value comes from a scenario script, not sensing
