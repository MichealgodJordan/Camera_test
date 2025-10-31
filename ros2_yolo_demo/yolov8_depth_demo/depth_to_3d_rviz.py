#!/usr/bin/env python3
"""Subscribe to /tracked_objects and /camera/depth/image_rect_raw and camera_info,
compute 3D positions and publish visualization markers for RViz.
Also publishes a /detected_objects_3d topic (JSON string) with 3D coords.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import json
import time

class DepthTo3DRViz(Node):
    def __init__(self):
        super().__init__('depth_to_3d_rviz')
        self.bridge = CvBridge()
        self.depth = None
        self.fx = self.fy = self.cx = self.cy = None
        self.sub_tracked = self.create_subscription(String, '/tracked_objects', self.tracked_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 4)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.pub_3d = self.create_publisher(String, '/detected_objects_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection_markers', 10)
        self.frame_id = 'camera_color_optical_frame'
        self.get_logger().info('depth_to_3d_rviz started')

    def info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg: Image):
        # depth image (uint16 or float32) passed through
        arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth = np.array(arr, copy=False)

    def tracked_cb(self, msg: String):
        if self.depth is None or self.fx is None:
            return
        now = time.time()
        data = json.loads(msg.data)
        tracks = data.get('tracks', [])
        markers = MarkerArray()
        out = []
        for i, t in enumerate(tracks):
            bbox = t['bbox']
            x1,y1,x2,y2 = map(int, bbox)
            cx = int((x1 + x2)/2)
            cy = int((y1 + y2)/2)
            # safe bounds check
            h,w = self.depth.shape[:2]
            cx = max(0, min(cx, w-1))
            cy = max(0, min(cy, h-1))
            depth_val = float(self.depth[cy, cx])
            if depth_val == 0 or np.isnan(depth_val):
                # try small patch median
                patch = self.depth[max(0,cy-2):min(h,cy+3), max(0,cx-2):min(w,cx+3)]
                if patch.size == 0:
                    continue
                depth_val = float(np.median(patch[patch>0])) if np.any(patch>0) else 0
                if depth_val == 0:
                    continue
            z = depth_val
            x = (cx - self.cx) * z / self.fx
            y = (cy - self.cy) * z / self.fy
            out.append({'id': t['id'], 'class_id': t['class_id'], 'confidence': t['confidence'], 'x': x, 'y': y, 'z': z})
            # create a marker (sphere) at the 3D location
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detections'
            m.id = int(t['id'])
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 0.9
            # color by id (deterministic)
            cid = (int(t['id']) % 6)
            colors = [
                (1.0,0.0,0.0),
                (0.0,1.0,0.0),
                (0.0,0.0,1.0),
                (1.0,1.0,0.0),
                (1.0,0.0,1.0),
                (0.0,1.0,1.0),
            ]
            r,g,b = colors[cid]
            m.color.r = r
            m.color.g = g
            m.color.b = b
            markers.markers.append(m)
        # publish marker array and 3D detections
        self.marker_pub.publish(markers)
        out_msg = String()
        out_msg.data = json.dumps({'timestamp': now, 'objects_3d': out})
        self.pub_3d.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthTo3DRViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
