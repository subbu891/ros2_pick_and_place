from enum import Enum, auto
import math

from geometry_msgs.msg import PoseStamped
from pnp_interfaces.action import PickPlace
from pnp_interfaces.msg import DetectedObjectArray
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pnp_task.sorting_logic import (Bin, ObjectTracker, Observation, pick_was_verified,
                                    select_next_target, Workspace)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class State(Enum):
    WAIT_FOR_SCENE = auto()
    SELECT = auto()
    PICKING = auto()
    DONE = auto()


class SorterNode(Node):

    def __init__(self):
        super().__init__('sorter')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('object_height', 0.04)
        self.declare_parameter('settle_time', 4.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('bin_colors', ['red', 'green', 'blue'])
        self.declare_parameter('bin_x', [0.30, 0.45, 0.60])
        self.declare_parameter('bin_y', [0.35, 0.35, 0.35])
        self.declare_parameter('workspace', [0.25, 0.75, -0.40, 0.22])

        p = self.get_parameter
        self.world_frame = p('world_frame').value
        self.object_height = p('object_height').value
        self.settle_time = p('settle_time').value
        self.max_retries = p('max_retries').value
        self.bins = {c: Bin(c, x, y) for c, x, y in
                     zip(p('bin_colors').value, p('bin_x').value, p('bin_y').value)}
        self.workspace = Workspace(*p('workspace').value)

        self.tracker = ObjectTracker()
        self.placed_per_bin = {c: 0 for c in self.bins}
        self.attempts = {}
        self.blacklist = set()
        self.current = None
        self.to_verify = None
        self.state = State.WAIT_FOR_SCENE
        self.state_since = self.get_clock().now()

        self.create_subscription(DetectedObjectArray, '/object_detector/objects',
                                 self.on_objects, 10)
        self.client = ActionClient(self, PickPlace, 'pick_place')
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'Sorter ready, bins: {list(self.bins)}')

    # ------------------------------------------------------------------ inputs
    def on_objects(self, msg):
        if self.state == State.PICKING:
            return   # the arm blocks the camera while it moves
        self.tracker.update([
            Observation(o.color, o.pose.pose.position.x, o.pose.pose.position.y,
                        o.pose.pose.position.z, yaw_from_quaternion(o.pose.pose.orientation))
            for o in msg.objects])

    # ----------------------------------------------------------- state machine
    def set_state(self, s):
        self.get_logger().info(f'{self.state.name} -> {s.name}')
        self.state = s
        self.state_since = self.get_clock().now()

    def elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9

    def tick(self):
        if self.state == State.WAIT_FOR_SCENE:
            if self.elapsed() >= self.settle_time and self.client.server_is_ready():
                if self.to_verify is not None:
                    self.verify(self.to_verify)
                    self.to_verify = None
                self.set_state(State.SELECT)

        elif self.state == State.SELECT:
            target = select_next_target(self.tracker.stable(), self.bins, self.workspace,
                                        self.blacklist)
            if target is not None:
                self.send_pick(target)
            elif self.elapsed() > 10.0:
                self.set_state(State.DONE)
                self.get_logger().info(
                    f'ALL DONE. Placed per bin: {self.placed_per_bin}, '
                    f'skipped: {sorted(self.blacklist) or "none"}')

    # ----------------------------------------------------------------- actions
    def pose(self, x, y, z, yaw):
        ps = PoseStamped()
        ps.header.frame_id = self.world_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        return ps

    def send_pick(self, t):
        sx, sy = self.bins[t.color].slot(self.placed_per_bin[t.color])
        goal = PickPlace.Goal()
        goal.object_id = t.track_id
        goal.object_height = float(self.object_height)
        goal.pick_pose = self.pose(t.x, t.y, t.z, t.yaw)
        goal.place_pose = self.pose(sx, sy, 0.0, 0.0)

        self.current = t
        self.set_state(State.PICKING)
        self.get_logger().info(
            f'Picking {t.track_id} at ({t.x:.3f}, {t.y:.3f}) -> {t.color} bin')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.finish(False, 'goal rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self.finish(f.result().result.success, f.result().result.message))

    def finish(self, success, message=''):
        t = self.current
        if success:
            self.to_verify = t   # checked by the camera in WAIT_FOR_SCENE
            self.get_logger().info(f'{t.track_id}: action succeeded, verifying with camera')
        else:
            self.record_failure(t, message)
        self.current = None
        self.set_state(State.WAIT_FOR_SCENE)

    def verify(self, t):
        if pick_was_verified(self.tracker, t.track_id):
            self.placed_per_bin[t.color] += 1
            self.tracker.remove(t.track_id)
            self.get_logger().info(f'Verified: {t.track_id} is in the {t.color} bin')
        else:
            self.record_failure(t, 'object still at pick location')

    def record_failure(self, t, message):
        n = self.attempts.get(t.track_id, 0) + 1
        self.attempts[t.track_id] = n
        self.get_logger().warn(f'{t.track_id} failed ({message}), attempt {n}')
        if n > self.max_retries:
            self.blacklist.add(t.track_id)
            self.get_logger().error(f'Giving up on {t.track_id}')


def main(args=None):
    rclpy.init(args=args)
    node = SorterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
