from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'antdrone_pcl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='support@ecoationlabs.ca',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz_pcl_frame_fixer = antdrone_pcl.gz_pcl_frame_fixer:main',
        ],
    },
)
