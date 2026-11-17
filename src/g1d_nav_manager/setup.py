from setuptools import find_packages, setup

package_name = "g1d_nav_manager"

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
    description="NavGPT-style navigation manager on Nav2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["nav_manager = g1d_nav_manager.nav_manager:main"]},
)
