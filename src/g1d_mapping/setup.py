from setuptools import find_packages, setup

package_name = "g1d_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hitliujianyun",
    maintainer_email="hitliujianyun@163.com",
    description="G1-D map saving",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["map_manager = g1d_mapping.map_manager:main"]},
)
