#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['lqr_control_states'],
    package_dir = {'': 'src'}
)

setup(**d)
