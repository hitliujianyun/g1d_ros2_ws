from glob import glob

from setuptools import find_packages, setup

package_name = "g1d_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hitliujianyun",
    maintainer_email="hitliujianyun@163.com",
    description="Closed-loop G1-D simulator",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["chassis_sim = g1d_sim.chassis_sim:main"]},
)
