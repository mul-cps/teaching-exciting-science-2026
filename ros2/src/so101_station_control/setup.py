from setuptools import find_packages, setup

package_name = "so101_station_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/moveit_resident.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CPS station maintainers",
    maintainer_email="cps@example.invalid",
    description="FollowJointTrajectory bridge to the resident Python SO-ARM101 bus owner.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "trajectory_server = so101_station_control.trajectory_server:main",
    ]},
)
