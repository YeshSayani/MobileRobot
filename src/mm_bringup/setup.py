from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'mm_bringup'

# If you don't have a python module dir named mm_bringup/, find_packages() returns []
# and that's OK for a bringup-only package.
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # or [] if you truly have no Python package dir
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/config', ['config/wheel_vel_test.yaml']),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup for mobile manipulator sim',
    license='Apache-2.0',
    entry_points={},  # none needed for this package
)

