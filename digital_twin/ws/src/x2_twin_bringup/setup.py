import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'x2_twin_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Digital Twin team',
    maintainer_email='burabayeva.aidana@gmail.com',
    description='Bring-up of the X2 stack against the digital twin.',
    license='Proprietary',
    tests_require=['pytest'],
)
