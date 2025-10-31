#!/usr/bin/env python3
"""YOLOv8 detector + lightweight tracker node.
Subscribes to /camera/color/image_raw, publishes /detected_objects (JSON) and /tracked_objects (JSON).
Will try to use a SORT-style tracker (filterpy). If not available, falls back to a simple centroid-based tracker.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np
import time

# Try to import ultralytics; user must install it in their environment
try:
    from ultralytics import YOLO
except Exception as e:
    YOLO = None

# Try to import a tracker (simple fallback implemented below)
try:
    from filterpy.kalman import KalmanFilter
    has_filterpy = True
except Exception:
    has_filterpy = False

class SimpleCentroidTracker:
    """Very lightweight tracker that assigns IDs by nearest centroid.
    Not robust but easy to run without extra deps.
    """
    def __init__(self, max_distance=50):
        self.next_id = 0
        self.objects = {}  # id -> centroid
        self.max_distance = max_distance

    def update(self, bboxes):
        # bboxes: list of [x1,y1,x2,y2]
        centroids = [((b[0]+b[2])/2, (b[1]+b[3])/2) for b in bboxes]
        assignments = {}
        used_ids = set()
        results = []
        for c_idx, c in enumerate(centroids):
            best_id = None
            best_dist = None
            for oid, ocent in self.objects.items():
                dist = (ocent[0]-c[0])**2 + (ocent[1]-c[1])**2
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_id = oid
            if best_id is None or best_dist is None or best_dist > (self.max_distance**2):
                # new object
                oid = self.next_id
                self.next_id += 1
                self.objects[oid] = c
                results.append(oid)
            else:
                # assign existing
                self.objects[best_id] = c
                results.append(best_id)
                used_ids.add(best_id)
        # cleanup: remove objects not seen for a while is omitted for simplicity
        return results

class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 4)
        self.det_pub = self.create_publisher(String, '/detected_objects', 10)
        self.tracked_pub = self.create_publisher(String, '/tracked_objects', 10)
        self.get_logger().info('yolo_tracker node started')
        self.model = None
        if YOLO is not None:
            try:
                # Use a small model by default; user can replace with a custom weights path
                self.model = YOLO('yolov8n.pt')
                self.get_logger().info('Loaded yolov8n model')
            except Exception as e:
                self.get_logger().warning(f'Could not load YOLOv8 model: {e}')
                self.model = None
        else:
            self.get_logger().warning('ultralytics YOLO not available; install ultralytics to run detection')

        self.tracker = SimpleCentroidTracker(max_distance=60)
        self.last_time = time.time()
        self.fps = 0.0

    def image_cb(self, msg):
        # Measure FPS
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps = 0.9*self.fps + 0.1*(1.0/dt)
        self.last_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h,w = cv_image.shape[:2]

        detections = []
        tracked = []

        if self.model is not None:
            try:
                results = self.model(cv_image)[0]
                # results.boxes: list
                for box in results.boxes:
                    x1,y1,x2,y2 = map(float, box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    detections.append({'bbox':[x1,y1,x2,y2], 'confidence':conf, 'class_id':cls})
            except Exception as e:
                self.get_logger().warning(f'YOLO inference failed: {e}')
        else:
            # no model: publish empty
            pass

        # tracking using centroid tracker
        bboxes = [d['bbox'] for d in detections]
        ids = self.tracker.update(bboxes) if len(bboxes)>0 else []
        for i,d in enumerate(detections):
            tid = ids[i] if i < len(ids) else -1
            tracked.append({'id': int(tid), 'bbox': d['bbox'], 'confidence': d['confidence'], 'class_id': d['class_id']})

        # Publish detections and tracked objects as JSON strings
        det_msg = String()
        det_msg.data = json.dumps({'timestamp': now, 'fps': self.fps, 'detections': detections})
        self.det_pub.publish(det_msg)

        trk_msg = String()
        trk_msg.data = json.dumps({'timestamp': now, 'fps': self.fps, 'tracks': tracked})
        self.tracked_pub.publish(trk_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
