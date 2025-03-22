from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  	(os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanmcginnis26@gmail.com',
    description='TODO: Package description',
    license='All Rights Reserved',
    entry_points={
        'console_scripts': [
            'arm = arm_pkg.arm_node:main',
            'headless = arm_pkg.arm_headless:main'
        ],
    },
)
