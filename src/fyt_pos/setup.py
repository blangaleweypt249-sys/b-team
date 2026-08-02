from glob import glob
from setuptools import find_packages, setup


package_name = 'fyt_pos'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='upre',
    maintainer_email='upre@todo.todo',
    description='比赛场地定位、区域判定与 RViz 可视化。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'field_localizer = fyt_pos.field_localizer:main',
            'field_visualizer = fyt_pos.field_visualizer:main',
            'simple_position = fyt_pos.simple_position:main',
        ],
    },
)