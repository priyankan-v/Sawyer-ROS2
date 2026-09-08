#!/usr/bin/env python3
from setuptools import find_packages, setup


setup(
    name='intera_interface',
    version='5.3.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src'),
)
