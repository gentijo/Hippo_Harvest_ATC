from setuptools import find_packages, setup


package_name = "air_traffic_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "plugin.xml"]),
        ("share/" + package_name + "/launch", ["launch/traffic_manager.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hippo Harvest",
    maintainer_email="user@example.com",
    description="RQT panel for observing multi-robot traffic-control metrics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "traffic_manager_node = air_traffic_control.traffic_manager_node:main",
        ],
    },
)
