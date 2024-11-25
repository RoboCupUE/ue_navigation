from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trajectory_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/params.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernando',
    maintainer_email='fergonzaramos@yahoo.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_navigator_node = trajectory_navigator.trajectory_navigator_node:main'
        ],
    },
)
