from setuptools import find_packages, setup

package_name = 'headless_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David',
    maintainer_email='ds0196@uah.edu',
    description='Headless rover control package to handle command interpretation and embedded interfacing.',
    license='All Rights Reserved',
    entry_points={
        'console_scripts': [
            "headless_full = src.headless_node:main",
        ],
    },
)
