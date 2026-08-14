from glob import glob
from os.path import join

from setuptools import setup


package_name = "go2_nav2_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "config"), glob("config/*.yaml")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="GoGoGuard",
    maintainer_email="engineering@gogoguard.local",
    description="Version-pinned Nav2 FollowPath patrol runtime for Go2",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_runtime_manager = go2_nav2_runtime.patrol_runtime_manager:main",
            "runtime_trace_recorder = go2_nav2_runtime.runtime_trace_recorder:main",
            "runtime_diagnostics = go2_nav2_runtime.runtime_diagnostics:main",
        ],
    },
)
