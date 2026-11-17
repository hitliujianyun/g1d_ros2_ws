from glob import glob

from setuptools import find_packages, setup

package_name = "g1d_robot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hitliujianyun",
    maintainer_email="hitliujianyun@163.com",
    description="G1-D robot interface nodes",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chassis_restamp = g1d_robot_bridge.chassis_restamp:main",
            "body_state = g1d_robot_bridge.body_state:main",
            "chassis_events = g1d_robot_bridge.chassis_events:main",
        ],
    },
)
