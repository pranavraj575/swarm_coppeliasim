from distutils.core import setup
from setuptools import find_packages

setup(
    name='swarm_coppeliasim',
    version='6.9.0',
    packages=find_packages(),
    install_requires=['numpy',
                      'psutil',
                      'zmq',
                      'cbor',
                      'neat-python==0.92',
                     ],
    license='Liscence to Krill',
)
