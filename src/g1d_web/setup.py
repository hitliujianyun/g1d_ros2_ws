from glob import glob

from setuptools import find_packages, setup

package_name = "g1d_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # the server serves <package>/static (frontend incl. vendored three.js)
    package_data={package_name: ["static/*.html", "static/css/*", "static/js/*", "static/vendor/three/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="hitliujianyun",
    maintainer_email="hitliujianyun@163.com",
    description="Browser UI for the G1-D navigation stack",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["g1d_web = g1d_web.__main__:main"]},
)
