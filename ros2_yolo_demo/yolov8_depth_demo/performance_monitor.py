    #!/usr/bin/env python3
    """Monitor FPS (from detections), CPU, memory, and a simple depth-quality metric,
    publish them to /system_metrics and log to /tmp/yolo_metrics.csv for later plotting.
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import psutil
    import json
    import time
    import os

    class PerfMonitor(Node):
        def __init__(self):
            super().__init__('performance_monitor')
            self.sub_det = self.create_subscription(String, '/detected_objects', self.det_cb, 10)
            self.pub = self.create_publisher(String, '/system_metrics', 10)
            self.last_fps = 0.0
            self.log_path = '/tmp/yolo_metrics.csv'
            # write header
            if not os.path.exists(self.log_path):
                with open(self.log_path,'w') as f:
                    f.write('timestamp,fps,cpu_percent,mem_percent
')
            self.get_logger().info('performance_monitor started, logging to ' + self.log_path)

        def det_cb(self, msg):
            try:
                data = json.loads(msg.data)
                fps = data.get('fps', 0.0)
            except Exception:
                fps = 0.0
            cpu = psutil.cpu_percent(interval=None)
            mem = psutil.virtual_memory().percent
            ts = time.time()
            metrics = {'timestamp': ts, 'fps': fps, 'cpu_percent': cpu, 'mem_percent': mem}
            self.pub.publish(String(data=json.dumps(metrics)))
            # append to csv
            with open(self.log_path,'a') as f:
                f.write(f"{ts},{fps},{cpu},{mem}\n")

    def main(args=None):
        rclpy.init(args=args)
        node = PerfMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
