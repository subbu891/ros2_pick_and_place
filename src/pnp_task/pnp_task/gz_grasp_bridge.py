from functools import partial
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformException, TransformListener


def nearest(poses, x, y, z):
    """Return (name, distance) of the pose closest to (x, y, z)."""
    best, best_d = None, float('inf')
    for name, p in poses.items():
        d = math.dist(p, (x, y, z))
        if d < best_d:
            best, best_d = name, d
    return best, best_d


class GzGraspBridge(Node):

    def __init__(self):
        super().__init__('gz_grasp_bridge')
        self.declare_parameter('cubes', ['cube_red_0'])
        self.declare_parameter('tcp_frame', 'gripper_tcp')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('max_attach_distance', 0.06)
        self.cubes = list(self.get_parameter('cubes').value)
        self.tcp = self.get_parameter('tcp_frame').value
        self.world = self.get_parameter('world_frame').value
        self.max_d = self.get_parameter('max_attach_distance').value

        self.attach_pub = {c: self.create_publisher(Empty, f'/pnp/attach/{c}', 10)
                           for c in self.cubes}
        self.detach_pub = {c: self.create_publisher(Empty, f'/pnp/detach/{c}', 10)
                           for c in self.cubes}
        self.poses = {}
        self.held = None
        self.released = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for c in self.cubes:
            # partial() fixes the cube name, so one callback serves all cubes
            self.create_subscription(PoseStamped, f'/model/{c}/pose',
                                     partial(self.on_pose, c), 10)
        self.create_subscription(Bool, '/pnp/gripper/grasp', self.on_grasp, 10)
        self.create_timer(2.0, self.release_all_once)
        self.get_logger().info(f'Grasp bridge ready for {len(self.cubes)} cubes')

    def on_pose(self, name, msg):
        p = msg.pose.position
        self.poses[name] = (p.x, p.y, p.z)

    def release_all_once(self):
        # Wait until Gazebo publishes the cube poses, i.e. it is really running
        if self.released or len(self.poses) < len(self.cubes):
            return
        for c in self.cubes:
            self.detach_pub[c].publish(Empty())
        self.released = True
        self.get_logger().info('Released all cubes at startup')

    def on_grasp(self, msg):
        if not msg.data:
            if self.held:
                self.detach_pub[self.held].publish(Empty())
                self.get_logger().info(f'Detached {self.held}')
                self.held = None
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.world, self.tcp, Time())
        except TransformException as e:
            self.get_logger().error(f'No TCP transform: {e}')
            return
        t = tf.transform.translation
        name, d = nearest(self.poses, t.x, t.y, t.z)
        if name is None or d > self.max_d:
            self.get_logger().warn(f'No cube within {self.max_d} m of the TCP (nearest: {d:.3f} m)')
            return
        self.attach_pub[name].publish(Empty())
        self.held = name
        self.get_logger().info(f'Attached {name} ({d * 100:.1f} cm from TCP)')


def main(args=None):
    rclpy.init(args=args)
    node = GzGraspBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
