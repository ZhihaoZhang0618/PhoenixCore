from setuptools import find_packages, setup
import os
from glob import glob

package_name = "f1tenth_system"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),  # 包含 yaml 文件
        (
            os.path.join("share", package_name, "behaviour_trees"),
            glob(os.path.join("behaviour_trees", "*.xml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EleSheep",
    maintainer_email="ZhizhaoZhang@gmail.com",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            # Add console scripts here if needed
        ],
    },
)
