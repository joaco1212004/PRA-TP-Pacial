"""Install the custom cafe world, its models, its map and its launch file."""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "custom_map"


def generate_folder_data_files(folder_name):
    """Walk a directory recursively to map subfolders and files for setuptools."""
    data_files = []
    for path, _, files in os.walk(folder_name):
        if files:
            install_path = os.path.join("share", package_name, path)
            source_files = [os.path.join(path, f) for f in files]
            data_files.append((install_path, source_files))
    return data_files


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml", "README.md"]),
    (os.path.join("share", package_name, "launch"),
     glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
    (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
    (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*.world"))),
    (os.path.join("share", package_name, "maps"),
     glob(os.path.join("maps", "*.yaml"))
     + glob(os.path.join("maps", "*.yml"))
     + glob(os.path.join("maps", "*.pgm"))
     + glob(os.path.join("maps", "*.png"))),
]

# Recursively add all nested files inside models/ (e.g., custom_cafe/model.sdf)
if os.path.exists("models"):
    data_files.extend(generate_folder_data_files("models"))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="udesa",
    maintainer_email="tadeo.casiraghi@gmail.com",
    description="Custom cafe world, map and launch file for the ROSMASTER X3 simulation",
    license="BSD-3-Clause",
    # colcon only runs the tests in test/ when the package declares a test
    # dependency on pytest. setuptools dropped tests_require, so declaring it
    # there makes colcon fall back to `setup.py test`, which collects nothing
    # and reports the package as passing with zero tests.
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "feature_finder = custom_map.features:main",
            "landmark_observer = custom_map.feature_finder:main"
        ],
    },
)
