#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['franka_perception'],
    package_dir={'': 'src'},
    requires=['rospy', 'numpy', 'cv2', 'sklearn']
)

setup(**setup_args)
