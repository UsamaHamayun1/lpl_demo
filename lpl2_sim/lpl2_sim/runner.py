# Common main() for lpl2 nodes: spin until Ctrl+C / launch shutdown without tracebacks.
import signal

import rclpy
from rclpy.executors import ExternalShutdownException


def run(node_class, args=None, on_exit=None):
    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        # rclpy (Jazzy) can raise from take_message when SIGINT invalidates the
        # context mid-spin (the context may not report "shut down" yet). Swallow only
        # that known error, or any error once shutdown is under way.
        if node.context.ok() and 'Unable to convert call argument' not in str(e):
            raise
    finally:
        # Ctrl+C under ros2 launch arrives twice (terminal + launch); ignore the
        # second one so cleanup (e.g. finalising a video file) is not interrupted.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if on_exit is not None:
            on_exit(node)
        node.destroy_node()
        rclpy.try_shutdown()
