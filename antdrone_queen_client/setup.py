import os
from glob import glob
from setuptools import find_packages, setup

package_name = "antdrone_queen_client"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),

        ("share/" + package_name, ["package.xml"])
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ecoation Labs",
    maintainer_email="support@ecoationlabs.ca",
    description="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'rmf_client = antdrone_queen_client.rmf_client:main',
            'heartbeat = antdrone_queen_client.heartbeat:main',
        ],
    },
)