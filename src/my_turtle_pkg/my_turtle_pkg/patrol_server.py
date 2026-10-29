import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from my_turtle_interfaces.action import Patrol

class PatrolServer(Node):

    def __init__(self):
        super().__init__('patrol_server')
        self._server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_cb,
        )
        self.get_logger().info('patrol_server 已启动，等待目标')

    def execute_cb(self, goal_handle):
        n = len(goal_handle.request.waypoints)
        self.get_logger().info(f'收到假任务：{n} 个航点')

        result = Patrol.Result()
        result.success = True
        result.waypoints_completed = 0
        result.message = 'dummy'

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
