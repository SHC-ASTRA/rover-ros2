from setuptools import find_packages, setup

package_name = "headless_pkg"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="David Sharpe",
    maintainer_email="ds0196@uah.edu",
    description="Provides headless rover control, similar to Basestation.",
    license="AGPL-3.0-only",
    entry_points={
        "console_scripts": ["headless_full = src.headless_node:main"],
    },
)
