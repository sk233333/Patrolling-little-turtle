import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node

from my_turtle_interfaces.action import Patrol

class PatrolClient(Node):

    def __init__(self):
        super().__init__('patrol_client')
        self._client = ActionClient(self, Patrol, 'patrol')
        self._done = False

    def send_dummy(self):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('patrol_server 未就绪，5 秒超时')
            self._done = True
            return
        
        goal = Patrol.Goal()
        goal.waypoints = [Point(x=1.0, y=1.0, z=0.0)]
        self.get_logger().info(f'发送 {len(goal.waypoints) } 个航点')

        self._client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            self._done = True
            return
        
        self.get_logger().info('目标已接受，等待结果')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f'结果：success={result.success},'
            f'waypoints_completed={result.waypoints_completed},'
            f'message={result.message}'
        )
        self._done = True

def main(args=None):
    rclpy.init(args=args)
    node = PatrolClient()
    try:
        node.send_dummy()
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()