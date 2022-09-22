#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    install_requires=["whiptail==0.2"],
    packages=['rasberry_coordination', 'rasberry_health_monitoring_pkg', 'rasberry_uv_treatment_pkg', 'rasberry_data_collection_pkg'],
    package_dir={'': 'src'}
)

setup(**d)
