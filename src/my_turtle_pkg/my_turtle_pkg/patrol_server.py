import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose

from my_turtle_interfaces.action import Patrol

from my_turtle_pkg.patrol_math import clamp, normalize_angle, remaining_distance


class PatrolServer(Node):

    def __init__(self):
        super().__init__('patrol_server')

        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('heading_tolerance', 0.10)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('feedback_period', 0.2)
        self.declare_parameter('pose_timeout', 2.0)

        self._pose = None
        self._pose_event = threading.Event()
        self._busy = False

        self._pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self._pose_cb, 10,
            callback_group=ReentrantCallbackGroup())
        self._cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self._server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info('patrol_server 已启动，等待目标')

    def _goal_cb(self, goal_request):
        if self._busy:
            self.get_logger().warn('已有任务在执行，拒绝新目标')
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('收到取消请求，将在当前周期停车')
        return CancelResponse.ACCEPT

    def _pose_cb(self, msg):
        self._pose = msg
        self._pose_event.set()

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _execute_cb(self, goal_handle):
        result = Patrol.Result()
        waypoints = goal_handle.request.waypoints

        if not waypoints:
            self.get_logger().warn('收到空航点列表，拒绝执行')
            goal_handle.abort()
            result.success = False
            result.waypoints_completed = 0
            result.message = 'empty waypoints'
            return result

        if not self._pose_event.wait(timeout=float(self.get_parameter('pose_timeout').value)):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('等待位姿期间收到取消')
                goal_handle.canceled()
                result.success = False
                result.waypoints_completed = 0
                result.message = 'canceled before pose'
                return result
            self.get_logger().error('未收到位姿，超时 abort')
            goal_handle.abort()
            result.success = False
            result.waypoints_completed = 0
            result.message = 'pose timeout'
            return result

        wp = waypoints[0]
        feedback = Patrol.Feedback()
        feedback.total_waypoints = len(waypoints)
        feedback.current_index = 0

        linear_speed = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        self.get_logger().info(
            f'开始巡逻，速度 linear={linear_speed} m/s, angular={angular_speed} rad/s')

        last_feedback = time.monotonic()
        completed = 0

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    self.get_logger().warn(
                        f'任务已取消：完成 {completed}/{len(waypoints)} 个航点')
                    goal_handle.canceled()
                    result.success = False
                    result.waypoints_completed = completed
                    result.message = 'canceled'
                    return result

                pose = self._pose
                if pose is None:
                    time.sleep(0.01)
                    continue

                linear_speed = float(self.get_parameter('linear_speed').value)
                angular_speed = float(self.get_parameter('angular_speed').value)
                heading_gain = float(self.get_parameter('heading_gain').value)
                waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
                heading_tolerance = float(self.get_parameter('heading_tolerance').value)
                feedback_period = float(self.get_parameter('feedback_period').value)

                dx = wp.x - pose.x
                dy = wp.y - pose.y
                dist = math.hypot(dx, dy)
                err = normalize_angle(math.atan2(dy, dx) - pose.theta)

                if dist <= waypoint_tolerance:
                    self._stop()
                    self.get_logger().info(
                        f'到达航点 1/{len(waypoints)}，累计完成 1 个')
                    completed = 1
                    result.success = True
                    result.waypoints_completed = 1
                    result.message = 'reached'
                    goal_handle.succeed()
                    return result

                if abs(err) > heading_tolerance:
                    v = 0.0
                else:
                    v = linear_speed
                w = clamp(heading_gain * err, -angular_speed, angular_speed)
                self._publish(v, w)

                now = time.monotonic()
                if now - last_feedback >= feedback_period:
                    feedback.distance_to_target = float(dist)
                    feedback.distance_remaining = float(
                        remaining_distance(pose, waypoints, 0))
                    goal_handle.publish_feedback(feedback)
                    last_feedback = now

                time.sleep(1.0 / float(self.get_parameter('control_frequency').value))
        except Exception as exc:
            self._stop()
            self.get_logger().error(f'执行异常：{exc}')
            goal_handle.abort()
            result.success = False
            result.waypoints_completed = completed
            result.message = f'error: {exc}'
            return result
        finally:
            self._stop()
            self._busy = False

def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
