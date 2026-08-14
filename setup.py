from setuptools import setup


PACKAGES = {
    "gogoguard_contracts": "modules/contracts/gogoguard_contracts",
    "gogoguard_calibration": "modules/calibration/gogoguard_calibration",
    "gogoguard_device_io": "modules/device_io/gogoguard_device_io",
    "gogoguard_interaction": "modules/interaction/gogoguard_interaction",
    "gogoguard_inspection": "modules/inspection/gogoguard_inspection",
    "gogoguard_mission": "modules/mission/gogoguard_mission",
    "gogoguard_data_capture": "modules/data_capture/gogoguard_data_capture",
    "gogoguard_evidence": "modules/evidence/gogoguard_evidence",
    "gogoguard_transfer": "modules/transfer/gogoguard_transfer",
    "gogoguard_map_factory": "services/map_factory/gogoguard_map_factory",
    "gogoguard_interaction_edge": "services/interaction_edge/gogoguard_interaction_edge",
    "gogoguard_platform_edge": "services/platform_edge/gogoguard_platform_edge",
    "gogoguard_route": "modules/route/gogoguard_route",
    "gogoguard_navigation": "modules/navigation/gogoguard_navigation",
    "gogoguard_site_console": "apps/site_console/backend/gogoguard_site_console",
    "gogoguard_field_workstation": "apps/field_workstation/backend/gogoguard_field_workstation",
}


setup(
    name="gogoguard-robot-inspection-v2",
    version="0.1.0",
    description="Go2 inspection mapping and runtime V2",
    python_requires=">=3.9",
    packages=list(PACKAGES),
    package_dir=PACKAGES,
    package_data={"gogoguard_calibration": ["config/*.json"]},
    entry_points={
        "console_scripts": [
            "gogoguard-site-console=gogoguard_site_console.__main__:main",
            "gogoguard-field-workstation=gogoguard_field_workstation.__main__:main",
            "gogoguard-mount-tf=gogoguard_calibration.mount_tf_publisher:main",
            "gogoguard-navigation-observer=gogoguard_navigation.observer:main",
            "gogoguard-navigation-supervisor=gogoguard_navigation.supervisor:main",
            "gogoguard-incident-recorder=gogoguard_evidence.incident_recorder:main",
            "gogoguard-interaction-edge=gogoguard_interaction_edge.__main__:main",
            "gogoguard-platform-edge=gogoguard_platform_edge.__main__:main",
            "gogoguard-platform-pose-stream=gogoguard_platform_edge.pose_stream:main",
            "gogoguard-battery-observer=gogoguard_device_io.battery_status:main",
        ]
    },
)
