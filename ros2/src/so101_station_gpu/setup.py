from setuptools import find_packages, setup


package_name = "so101_station_gpu"

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
    maintainer="CPS station maintainers",
    maintainer_email="cps@example.invalid",
    description="GPU-only GLX preflight and launcher for station RViz.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "rviz_gpu_guard = so101_station_gpu.gpu_renderer:main",
    ]},
)
