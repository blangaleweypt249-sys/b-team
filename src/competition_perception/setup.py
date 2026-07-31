from glob import glob
from setuptools import find_packages, setup


package_name = 'competition_perception'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 安装 ROS 2 包索引、参数文件和启动文件，供 ros2 launch 在部署机查找。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='upre',
    maintainer_email='upre@todo.todo',
    description='比赛场地定位、RGB-D 视觉感知与操作员可视化。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'field_localizer = competition_perception.field_localizer:main',
            'vision_detector = competition_perception.vision_detector:main',
            'field_visualizer = competition_perception.field_visualizer:main',
        ],
    },
)