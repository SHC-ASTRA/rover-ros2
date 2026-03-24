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
    description="Relays topics related to Biosensor between VicCAN (through Anchor) and basestation.",
    license="AGPL-3.0-only",
    entry_points={
        "console_scripts": ["bio = bio_pkg.bio_node:main"],
    },
)
