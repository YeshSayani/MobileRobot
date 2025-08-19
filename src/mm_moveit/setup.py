from setuptools import setup, find_packages
from glob import glob

package_name = 'mm_moveit'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='MoveIt bringup for mobile manipulator',
    license='Apache-2.0',
)

