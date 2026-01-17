#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped

class HumanTFPublisher(Node):
    def __init__(self):
        super().__init__('human_tf_publisher')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('follow_height', 5.0)
        self.declare_parameter('camera_offset_x', 0.0)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.015)
        self.declare_parameter('camera_fov_horizontal', 1.047)  # rad
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('default_human_z', 0.03)  # przybliżona wysokość człowieka (obrazka na symulacji!)
        self.declare_parameter('forward_scale', 1.0)     # skalowanie ruchu do przodu (z markera -> x body)

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.follow_height = self.get_parameter('follow_height').get_parameter_value().double_value
        self.camera_offset_x = self.get_parameter('camera_offset_x').get_parameter_value().double_value
        self.camera_offset_y = self.get_parameter('camera_offset_y').get_parameter_value().double_value
        self.camera_offset_z = self.get_parameter('camera_offset_z').get_parameter_value().double_value
        self.camera_fov_horizontal = self.get_parameter('camera_fov_horizontal').get_parameter_value().double_value
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.default_human_z = self.get_parameter('default_human_z').get_parameter_value().double_value
        self.forward_scale = self.get_parameter('forward_scale').get_parameter_value().double_value


        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub_position = self.create_publisher(Point, '/human_position_xy', 10)

        self.sub_human_array = self.create_subscription(
            MarkerArray,
            '/human_detection_markers',
            self.human_cb_array,
            10
        )

        # Subskrypcja pozycji lokalnej drona
        self.current_pose = PoseStamped()
        self.sub_local_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            10
        )

        self.get_logger().info(
            f"Node initialized. Publikuję TF 'human' w ramce {self.target_frame} "
            f"z follow_height={self.follow_height}, camera_offset_z={self.camera_offset_z} "
            f"oraz współrzędne na /human_position_xy"
        )

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def human_cb_array(self, msg: MarkerArray):
        if not msg.markers:
            return
        marker = msg.markers[0]  # bierzemy pierwszego człowieka
        self.publish_human_tf(marker)
        self.publish_human_position(marker)

    def publish_human_tf(self, marker: Marker):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target_frame
        t.child_frame_id = 'human'

        # Marker jest w base_link, dodajemy offset kamery
        x = marker.pose.position.x + self.camera_offset_x
        y = marker.pose.position.y + self.camera_offset_y
        z = self.default_human_z + self.camera_offset_z + self.follow_height

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

     
        t.transform.rotation.x = marker.pose.orientation.x
        t.transform.rotation.y = marker.pose.orientation.y
        t.transform.rotation.z = marker.pose.orientation.z
        t.transform.rotation.w = marker.pose.orientation.w

   
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Publikuję TF human: x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )


    def publish_human_position(self, marker: Marker):
        dy_body = marker.pose.position.y + self.camera_offset_y 
        dx_body = self.forward_scale * (marker.pose.position.z)

     
        yaw = 0.0
        if self.current_pose and self.current_pose.pose.orientation:
            _, _, yaw = self.quaternion_to_euler(self.current_pose.pose.orientation)

        # Obrót do ENU
        dx_enu = dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
        dy_enu = dx_body * math.sin(yaw) + dy_body * math.cos(yaw)

        pt = Point()
        pt.x = dx_enu
        pt.y = dy_enu
        pt.z = self.default_human_z + self.camera_offset_z + self.follow_height

        self.pub_position.publish(pt)
        self.get_logger().info(
            f"Publikuję pozycję człowieka (ENU względna): x={pt.x:.2f}, y={pt.y:.2f}"
        )


def main():
    rclpy.init()
    node = HumanTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Przerwano ręcznie")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
