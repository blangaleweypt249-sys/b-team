from setuptools import setup

package_name = 'ball_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perception_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/perception_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='husky',
    maintainer_email='husky@todo.todo',
    description='金色足球相机-雷达融合感知节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_node = ball_perception.usb_camera_node:main',
            'ball_distance_node = ball_perception.ball_distance_node:main',
        ],
    },
)
