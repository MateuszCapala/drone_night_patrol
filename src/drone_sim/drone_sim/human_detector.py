#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json


class YoloHumanDetector(Node):
    def __init__(self):
        super().__init__('yolov8_human_detector')

        # --- PARAMETRY KONFIGURACYJNE ---
        #self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_topic', '/color/image_raw')
        
        self.declare_parameter('publish_topic', '/human_detection')
        self.declare_parameter('visualization_topic', '/human_detection_markers')
        self.declare_parameter('annotated_image_topic', '/human_detection_image')
        self.declare_parameter('model_path', '/home/mateusz/Desktop/sem2_magisterka/drony/drone_night_patrol/yolo_thermal/person_detection/weights/best.pt')  # <-- podmień na swój model
        self.declare_parameter('frame_skip', 5)  # co ile klatek analizować
        self.declare_parameter('camera_frame_id', 'camera_link')  # frame ID dla wizualizacji

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        visualization_topic = self.get_parameter('visualization_topic').get_parameter_value().string_value
        annotated_image_topic = self.get_parameter('annotated_image_topic').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.frame_skip = self.get_parameter('frame_skip').get_parameter_value().integer_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value

        # --- Inicjalizacja modelu YOLO ---
        self.get_logger().info(f'Ładowanie modelu YOLOv8 z: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()
        self.frame_count = 0


        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(String, publish_topic, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, visualization_topic, 10)
        self.annotated_image_publisher = self.create_publisher(Image, annotated_image_topic, 10)

        self.get_logger().info('Node YOLOv8 Human Detector uruchomiony.')
        self.get_logger().info(f'Publikacja wizualizacji na: {visualization_topic}')
        self.get_logger().info(f'Publikacja obrazów z detekcjami na: {annotated_image_topic}')

    def image_callback(self, msg):
        self.frame_count += 1

        # Pomijaj niektóre klatki
        if self.frame_count % self.frame_skip != 0:
            return

        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated_image = cv_image.copy()

        # Wykrywanie
        results = self.model(cv_image)

        detections = []
        markers = MarkerArray()
        marker_id = 0

        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    if r.names[cls] == 'person' and conf > 0.5:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detections.append({
                            'label': 'person',
                            'confidence': conf,
                            'bbox': [x1, y1, x2, y2]
                        })

                        
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(annotated_image, f'Person: {conf:.2f}', 
                                  (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            
                        marker = self.create_detection_marker(x1, y1, x2, y2, conf, marker_id, msg.header.stamp)
                        markers.markers.append(marker)
                        marker_id += 1

        if detections:
            msg_out = {
                'detected': True,
                'detections': detections
            }
        else:
            msg_out = {'detected': False}

        msg_json = json.dumps(msg_out)
        self.publisher.publish(String(data=msg_json))

        self.marker_publisher.publish(markers)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_image_publisher.publish(annotated_msg)

        if detections:
            self.get_logger().info(f'Wykryto {len(detections)} osób')

    def create_detection_marker(self, x1, y1, x2, y2, confidence, marker_id, timestamp):
        """Tworzy marker wizualizacyjny dla detekcji w RViz"""
        marker = Marker()
        marker.header.frame_id = self.camera_frame_id
        marker.header.stamp = timestamp
        marker.ns = "human_detections"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        norm_x = (center_x - 320) / 320.0
        norm_y = (center_y - 240) / 240.0
        
        distance = 5.0
        marker.pose.position.x = distance
        marker.pose.position.y = -norm_x * distance * 0.5  
        marker.pose.position.z = -norm_y * distance * 0.5  

        marker.pose.orientation.w = 1.0

        width = (x2 - x1) / 100.0  # skalowanie
        height = (y2 - y1) / 100.0
        marker.scale.x = 0.2
        marker.scale.y = width
        marker.scale.z = height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = confidence

        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = YoloHumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
