from glob import glob
from os.path import join
from setuptools import setup


package_name = "go2_mapping_capture"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GoGoGuard",
    maintainer_email="devnull@example.com",
    description="Exclusive MID360 PointCloud2 mapping capture source",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "mount_calibration_recorder = go2_mapping_capture.mount_calibration_recorder:main",
        ],
    },
)
