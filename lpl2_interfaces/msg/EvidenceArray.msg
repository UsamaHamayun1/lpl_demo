# Full evidence state (every item, every time), published by the evidence layer.
std_msgs/Header header
uint32 seq               # increases by one per message; consumers drop out-of-order arrivals
EvidenceItem[] items
