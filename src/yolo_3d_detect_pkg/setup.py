from glob import glob
from setuptools import find_packages, setup


package_name = 'yolo_3d_detect_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/model', glob('model/*.pt')),
        ('share/' + package_name + '/model', glob('model/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='upre',
    maintainer_email='upre@todo.todo',
    description='比赛灵石与大地块 RGB-D 视觉检测。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_detector = yolo_3d_detect_pkg.vision_detector:main',
            'block_detector = yolo_3d_detect_pkg.block_detector:main',
        ],
    },
)