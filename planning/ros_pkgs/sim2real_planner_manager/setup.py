#!/usr/bin/env python3

from setuptools import setup

try:
    from catkin_pkg.python_setup import generate_distutils_setup
except ImportError:
    package_args = {
        "packages": ["sim2real_planner_manager"],
        "package_dir": {"": "src"},
    }
else:
    package_args = generate_distutils_setup(
        packages=["sim2real_planner_manager"],
        package_dir={"": "src"},
    )

setup(**package_args)
