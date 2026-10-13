# Drop-in stand-in for gazebo_msgs/ModelStates on Gazebo Harmonic.
# gz_world_state publishes entity poses as a TFMessage on MODEL_STATES_TOPIC;
# from_tf() turns that back into an object with the same .name / .pose lists the
# Gazebo Classic message had, so msg.name.index(...) and msg.pose[i].position work.
from geometry_msgs.msg import Pose

MODEL_STATES_TOPIC = '/lpl/model_states'


class ModelStates:
    def __init__(self):
        self.name = []
        self.pose = []


def from_tf(tf_msg):
    states = ModelStates()
    for t in tf_msg.transforms:
        p = Pose()
        p.position.x = t.transform.translation.x
        p.position.y = t.transform.translation.y
        p.position.z = t.transform.translation.z
        p.orientation = t.transform.rotation
        states.name.append(t.child_frame_id)
        states.pose.append(p)
    return states
