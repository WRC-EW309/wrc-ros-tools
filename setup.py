from setuptools import setup

package_name = 'wrc_ros_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arty',
    maintainer_email='dawkins@usna.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pose_listener = wrc_ros_tools.pose_listener:main',
                   'pose_listener_qualisys = wrc_ros_tools.pose_listener_qualisys:main',
                   'rosbag_service = wrc_ros_tools.rosbag_service:main',
                   'rigid_bodies_repub = wrc_ros_tools.rigid_bodies_republisher:main'
        ],
    },
)
