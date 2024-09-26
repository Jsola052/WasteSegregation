from setuptools import find_packages, setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python-headless',
        'scikit-learn',
        'pyrealsense2',
        'open3d',
        'ros2_numpy',
        'cv_bridge',
        'rclpy',
        'tf2_ros',
        'tf2_geometry_msgs'
    ],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='jsola052@fiu.edu',
    description='Package for detecting and transforming coordinates of green balls using ROS 2 and Intel RealSense camera.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ball_detection_node = camera_pkg.detect_green_ball_publisher:main",
            "detect_green_ball_subscriber = camera_pkg.detect_green_ball_subscriber:main",
            "coordinate_transformer = camera_pkg.coordinate_transformer:main",
            "transform_point_service = camera_pkg.transform_point_server:main",  
        ],
    },
)
