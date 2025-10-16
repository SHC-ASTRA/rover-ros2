from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_description'

list_of_folders = ["config", "launch", "meshes", "textures", "urdf"]

setup_data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
]

for folder in list_of_folders:
    setup_data_files.append((os.path.join('share', package_name, folder), glob(folder + '/*')))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=setup_data_files,  # created above with for loop
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Sharpe',
    maintainer_email='ds0196@uah.edu',
    description='This package contains configuration data, 3D models and launch files for Clucky\'s arm',
    license='BSD'
)
