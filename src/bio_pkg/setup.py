from setuptools import find_packages, setup

package_name = "bio_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tristan",
    maintainer_email="tristanmcginnis26@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": ["bio = bio_pkg.bio_node:main"],
    },
)
