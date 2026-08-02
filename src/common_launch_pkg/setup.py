from glob import glob
from setuptools import find_packages, setup


package_name = 'common_launch_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='upre',
    maintainer_email='upre@todo.todo',
    description='比赛系统总启动包。',
    license='Apache-2.0',
)