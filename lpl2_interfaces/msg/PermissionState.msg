# One decision permission in the authority envelope, as decided by LPL.

# Zero is UNAVAILABLE on purpose: a default-constructed permission must never read as usable.
uint8 UNAVAILABLE=0
uint8 AVAILABLE=1        # supported by evidence
uint8 HUMAN_GRANTED=2    # a human authorized it although evidence does not support it
uint8 EGRESS_ONLY=3      # revoked while the robot was inside its region: leaving is the only allowed use

uint8 PROV_EVIDENCE=0    # state follows the evidence rule
uint8 PROV_HUMAN_GRANT=1
uint8 PROV_DWELL_PENDING=2   # evidence is back but the restore dwell has not passed yet
uint8 PROV_STALE=3       # required evidence is missing or expired
uint8 PROV_FIXED=4       # not evidence-conditioned (always available) or forced by mode

string id                # "P3"
string name              # "Borrow oncoming lane (passing window)"
string region_id         # map region this permission governs, empty if none
uint8 state
bool evidence_supported  # the rule's evidence condition currently holds
uint8 provenance
string[] evidence_ids    # evidence the rule depends on
string reason            # why it is in this state, for the UI and logs
builtin_interfaces/Time since
bool human_grantable
string grant_id          # active human grant, empty if none
builtin_interfaces/Time grant_expires_at
