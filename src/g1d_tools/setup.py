from setuptools import find_packages, setup

package_name = "g1d_tools"

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
    description="G1-D field tools",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "check_interfaces = g1d_tools.check_interfaces:main",
            "cmd_vel_probe = g1d_tools.cmd_vel_probe:main",
            "acceptance = g1d_tools.acceptance:main",
            "calib_extrinsics = g1d_tools.calib_extrinsics:main",
            "map_compare = g1d_tools.map_compare:main",
            "nav_cli = g1d_tools.nav_cli:main",
        ],
    },
)
