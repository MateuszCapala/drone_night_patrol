#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math
import time

class MoveImage(Node):
    def __init__(self):
        super().__init__('move_image_node')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Czekam na usługę /gazebo/set_entity_state...')

        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        t = time.time() - self.start_time
        # Ścieżka okrężna
        r = 2.0
        x = r * math.cos(0.5 * t)
        y = r * math.sin(0.5 * t)
        z = 0.5

        state = EntityState()
        state.name = 'moving_image'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.w = 1.0

        req = SetEntityState.Request()
        req.state = state

        self.client.call_async(req)

def main():
    rclpy.init()
    node = MoveImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
