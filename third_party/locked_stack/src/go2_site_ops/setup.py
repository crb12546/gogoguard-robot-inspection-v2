from glob import glob
from os.path import join

from setuptools import setup


package_name = "go2_site_ops"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GoGoGuard",
    maintainer_email="engineering@gogoguard.local",
    description="Go2 site delivery, coordinate calibration, and map tooling",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "validate_coordinate_contract = go2_site_ops.validate_contract:main",
            "validate_mount_calibration = go2_site_ops.validate_calibration:main",
            "mount_tf_publisher = go2_site_ops.mount_tf_publisher:main",
            "site_console_capture_adapter = go2_site_ops.site_console_capture_adapter:main",
            "install_runtime_release = go2_site_ops.runtime_release_installer:main",
        ],
    },
)
