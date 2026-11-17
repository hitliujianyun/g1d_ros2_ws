from setuptools import find_packages, setup

package_name = "g1d_localization"

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
    description="G1-D localization nodes",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "lio_bridge = g1d_localization.lio_bridge:main",
        "localization_monitor = g1d_localization.localization_monitor:main",
    ]},
)
