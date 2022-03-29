#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['fkie_iop_cfg_ocu'],
    package_dir={'': 'src'},
    install_requires=['rospkg', 'yaml'],
    )

setup(**d)