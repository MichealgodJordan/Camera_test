from setuptools import setup

package_name = 'yolov8_depth_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv8 + Depth -> 3D demo with RViz visualization, tracking and performance monitor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_tracker = yolov8_depth_demo.yolo_tracker:main',
            'depth_to_3d_rviz = yolov8_depth_demo.depth_to_3d_rviz:main',
            'performance_monitor = yolov8_depth_demo.performance_monitor:main',
        ],
    },
)
