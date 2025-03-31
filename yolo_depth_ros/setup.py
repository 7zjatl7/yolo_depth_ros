from setuptools import find_packages, setup

package_name = 'yolo_depth_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Donghoon Lee',
    maintainer_email='7zjatl7@naver.com',
    description='ROS2 package for YOLO-based detection, depth estimation, tracking and debugging',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "goal_pub = yolo_depth_ros.goal_pub:main",
            # "img_pub = yolo_depth_ros.img_pub:main",
            "yolo_node = yolo_depth_ros.yolo_node:main",
            "depth_node = yolo_depth_ros.depth_node:main",
            "tracking_node = yolo_depth_ros.tracking_node:main",
            "yolo_depth_node = yolo_depth_ros.yolo_depth_node:main",
            "debug_node = yolo_depth_ros.debug_node:main",
        ],
    },
)
