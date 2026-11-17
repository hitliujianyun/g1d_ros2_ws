from setuptools import find_packages, setup

package_name = "g1d_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    extras_require={"test": ["pytest", "xacro"]},
    zip_safe=True,
    maintainer="hitliujianyun",
    maintainer_email="hitliujianyun@163.com",
    description="ROS-independent algorithms of the G1-D navigation stack",
    license="Apache-2.0",
    tests_require=["pytest"],
)
