from setuptools import find_packages, setup

package_name = 'ur_py_pkg'

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
    maintainer='robotics',
    maintainer_email='jsola052@fiu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tool_changer_server = ur_py_pkg.tool_changer_server:main",
            "vial_detection_node = ur_py_pkg.vial_detection_node:main",
            "vial_pickup_node = ur_py_pkg.vial_pickup_node:main",
            "sam_testing = ur_py_pkg.sam_testing:main",
            "bottles_demo_node = ur_py_pkg.bottles_demo_node:main"
        ],
    },
)
