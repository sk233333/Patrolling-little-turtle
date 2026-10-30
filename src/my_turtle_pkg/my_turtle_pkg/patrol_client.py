import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from my_turtle_interfaces.action import Patrol


class PatrolClient(Node):

    def __init__(self):
        super().__init__('patrol_client')
        self._client = ActionClient(self, Patrol, 'patrol')
        self._done = False
        self._goal_handle = None
        self._result_received = False
        self._sigint = False
        self._canceling = False
        self._goal_sent_at = None

        self.declare_parameter('default_waypoints_x', [2.0, 2.0, 8.0, 8.0])
        self.declare_parameter('default_waypoints_y', [2.0, 8.0, 8.0, 2.0])
        self.declare_parameter('goal_timeout_sec', 0.0)
        self.declare_parameter('server_wait_timeout', 10.0)
        self.declare_parameter('exit_after_result', True)

        signal.signal(signal.SIGINT, self._on_sigint)
        self.create_timer(0.1, self._tick)

    def send_patrol(self, waypoints):
        wait_timeout = float(self.get_parameter('server_wait_timeout').value)
        if not self._client.wait_for_server(timeout_sec=wait_timeout):
            self.get_logger().error(f'patrol_server 未就绪，{wait_timeout:.1f}s 超时')
            self._done = True
            return
        
        goal = Patrol.Goal()
        goal.waypoints = [Point(x=float(x), y=float(y), z=0.0)
                          for x, y in waypoints]
        self.get_logger().info(f'发送 {len(goal.waypoints)} 个航点')

        self._goal_sent_at = time.monotonic()
        self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        ).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            self._done = True
            return
        
        self._goal_handle = goal_handle
        self.get_logger().info('目标已接受，等待结果')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'反馈 [{fb.current_index + 1}/{fb.total_waypoints}] '
            f'到目标 {fb.distance_to_target:.3f}m')

    def _on_result(self, future):
        self._result_received = True
        result = future.result().result
        self.get_logger().info(
            f'结果：success={result.success},'
            f'waypoints_completed={result.waypoints_completed},'
            f'message={result.message}'
        )
        self._done = True
        if bool(self.get_parameter('exit_after_result').value):
            rclpy.shutdown()

    def _on_sigint(self, signum, frame):
        self._sigint = True

    def _elapsed(self):
        if self._goal_sent_at is None:
            return 0.0
        return time.monotonic() - self._goal_sent_at

    def _tick(self):
        goal_timeout = float(self.get_parameter('goal_timeout_sec').value)

        if self._sigint and not self._canceling:
            if self._result_received or self._goal_handle is None:
                rclpy.shutdown()
                return
            self._canceling = True
            self.get_logger().warn('收到 Ctrl+C，取消巡逻任务')
            self._goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_sent)
            return

        if (goal_timeout > 0.0 and self._goal_handle is not None
                and not self._canceling
                and self._elapsed() > goal_timeout):
            self._canceling = True
            self.get_logger().warn(f'任务超时 {goal_timeout:.1f}s，取消任务')
            self._goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_sent)

    def _on_cancel_sent(self, future):
        response = future.result()
        self.get_logger().info(
            f'取消请求已被受理：{len(response.goals_canceling)} 个目标')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PatrolClient()

    cli_args = remove_ros_args(args=sys.argv)
    if len(cli_args) >= 2:
        raw = cli_args[1]
        waypoints = [tuple(float(v) for v in pair.split(','))
                     for pair in raw.split(';')]
    else:
        xs = list(node.get_parameter('default_waypoints_x').value)
        ys = list(node.get_parameter('default_waypoints_y').value)
        if len(xs) != len(ys):
            node.get_logger().error('航点 x/y 数量不一致')
            xs, ys = [], []
        waypoints = list(zip(xs, ys))

    try:
        node.send_patrol(waypoints)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()