import sys
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (ConfigureController, ListControllers,
                                         LoadController, SwitchController)
import rclpy
from rclpy.node import Node


class ControllerActivator(Node):

    def __init__(self):
        super().__init__('controller_activator')
        self.declare_parameter(
            'controllers', ['joint_state_broadcaster', 'arm_controller', 'gripper_controller'])
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('timeout', 300.0)   # give up after this many seconds
        self.controllers = list(self.get_parameter('controllers').value)
        cm = self.get_parameter('controller_manager').value
        self.timeout = self.get_parameter('timeout').value

        self.list_cli = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.load_cli = self.create_client(LoadController, f'{cm}/load_controller')
        self.config_cli = self.create_client(ConfigureController, f'{cm}/configure_controller')
        self.switch_cli = self.create_client(SwitchController, f'{cm}/switch_controller')

    def call(self, client, request, timeout=30.0):
        """Call a service and wait for the answer; None if it doesn't come."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def states(self):
        result = self.call(self.list_cli, ListControllers.Request())
        return {} if result is None else {c.name: c.state for c in result.controller}

    def step(self, name, state):
        """Move one controller one step closer to 'active'."""
        if state is None:
            self.get_logger().info(f'Loading {name}')
            req = LoadController.Request()
            req.name = name
            self.call(self.load_cli, req)
        elif state == 'unconfigured':
            self.get_logger().info(f'Configuring {name}')
            req = ConfigureController.Request()
            req.name = name
            self.call(self.config_cli, req)
        elif state == 'inactive':
            self.get_logger().info(f'Activating {name}')
            req = SwitchController.Request()
            req.activate_controllers = [name]
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.timeout = Duration(sec=30)
            self.call(self.switch_cli, req)

    def run(self):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            states = self.states()
            pending = [c for c in self.controllers if states.get(c) != 'active']
            if states and not pending:
                self.get_logger().info(f'All controllers active: {self.controllers}')
                return True
            if pending:
                # Fix the first pending controller, then look at the real states again
                self.step(pending[0], states.get(pending[0]))
            time.sleep(1.0)
        self.get_logger().error(f'Timed out; still not active: {pending}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerActivator()
    ok = node.run()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)
