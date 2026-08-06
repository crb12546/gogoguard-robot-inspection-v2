from setuptools import setup

package_name = 'go2_fastlio_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GoGoGuard',
    maintainer_email='engineering@gogoguard.local',
    description='Route following and fail-closed motion safety for the Go2 patrol runtime.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_recorder = go2_fastlio_patrol.route_recorder:main',
            'waypoint_follower_go2_2 = go2_fastlio_patrol.waypoint_follower_go2_2:main',
            'unitree_safe_cmd_node = go2_fastlio_patrol.unitree_safe_cmd_node:main',
        ],
    },
)
