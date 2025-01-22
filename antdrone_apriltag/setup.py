import os
from glob import glob
from setuptools import find_packages, setup

package_name = "antdrone_apriltag"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ecoation Labs",
    maintainer_email="support@ecoationlabs.ca",
    description="gets drone under worker for pickup",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'gz_img_frame_fixer = antdrone_apriltag.gz_img_frame_fixer:main',
            'go_under = antdrone_apriltag.go_under:main',
            'frame_debug = antdrone_apriltag.frame_debug:main',
            'calibrate_img_publisher = antdrone_apriltag.calibrate_img_publisher:main',
        ],
    },
)