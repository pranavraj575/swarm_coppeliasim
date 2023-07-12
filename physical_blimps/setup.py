import os
import setuptools
import sys
from setuptools import setup, find_packages

def read(filename):
    return open(os.path.join(os.path.dirname(__file__), filename)).read()

setup(
    name = 'openBlimp',
    version='1.1',
    packages = find_packages('src'),
    package_dir = {'': 'src'},
    description = 'OpenBlimp Code and Workshop Demos',
    install_requires = []
)
