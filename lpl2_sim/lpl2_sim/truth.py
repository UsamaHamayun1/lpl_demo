# Helpers for the simulator ground-truth topic published by world_state.
import math

TOPIC = '/sim/model_poses'


def poses_by_name(msg):
    """TFMessage from /sim/model_poses -> {name: (x, y, yaw)}."""
    out = {}
    for t in msg.transforms:
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        out[t.child_frame_id] = (t.transform.translation.x, t.transform.translation.y, yaw)
    return out
