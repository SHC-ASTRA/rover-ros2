from setuptools import setup

package_name = "arm_pkg"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David",
    maintainer_email="ds0196@uah.edu",
    description="Relays topics related to the arm between VicCAN (through Anchor) and basestation.",
    license="AGPL-3.0-only",
    entry_points={
        "console_scripts": ["arm = arm_pkg.arm_node:main"],
    },
)
