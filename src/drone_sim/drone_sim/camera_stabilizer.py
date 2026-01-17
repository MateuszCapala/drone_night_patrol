#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool

import math
from scipy.spatial.transform import Rotation as R

class CameraStabilizer(Node):
    def __init__(self):
        super().__init__('camera_stabilizer')

        self.cli = self.create_client(SetLinkState, '/gazebo/set_link_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Czekam na serwis /gazebo/set_link_state...')

        self.timer = self.create_timer(0.02, self.update_camera)

        self.camera_link_name = 'realsense_camera::link'

        self.stable_pub = self.create_publisher(Bool, '/camera_stabilized', 10)

    def update_camera(self):
        req = SetLinkState.Request()
        req.link_state = LinkState()
        req.link_state.link_name = self.camera_link_name

        req.link_state.pose = Pose()

        r = R.from_euler('xyz', [0, -math.pi/2, 0])
        q = r.as_quat()
        req.link_state.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.cli.call_async(req)

        self.stable_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = CameraStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
