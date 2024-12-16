from setuptools import find_packages, setup

package_name = 'antdrone_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry = antdrone_localization.wheel_odometry:main',
            'amcl_visualizer = antdrone_localization.amcl_visualizer:main',
            'slam_image_recorder = antdrone_localization.slam_image_recorder:main',
            'slam_recording_publisher = antdrone_localization.slam_recording_publisher:main',
            'detected_dynamic_obstacles_publisher = antdrone_localization.detected_dynamic_obstacles_publisher:main',
            'gz_localization = antdrone_localization.gz_localization:main',
            'cmd_vel_scale_gz = antdrone_localization.cmd_vel_scale_gz:main',
            'gz_frame_name_fixer = antdrone_localization.gz_frame_name_fixer:main',

        ],
    },
)
