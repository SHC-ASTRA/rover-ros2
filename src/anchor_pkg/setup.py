from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = 'anchor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (path.join("share", package_name), ['package.xml']),
        (path.join("share", package_name, "launch"), glob("launch/*"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristanmcginnis26@gmail.com',
    description='Anchor node used to run all modules through a single modules MCU/Computer. Commands to all modules will be relayed through CAN',
    license='All Rights Reserved',
    entry_points={
        'console_scripts': [
            "anchor = anchor_pkg.anchor_node:main"
        ],
    },
)
