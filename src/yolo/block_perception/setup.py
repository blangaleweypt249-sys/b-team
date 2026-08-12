from setuptools import setup

package_name = 'block_perception'

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
    description='红蓝块 Orbbec RGB-D 相机检测与测距节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orbbec_camera_node = block_perception.orbbec_camera_node:main',
            'block_distance_node = block_perception.block_distance_node:main',
        ],
    },
)
