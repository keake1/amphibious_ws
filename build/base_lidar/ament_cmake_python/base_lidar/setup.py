from setuptools import find_packages
from setuptools import setup

setup(
    name='base_lidar',
    version='0.0.0',
    packages=find_packages(
        include=('base_lidar', 'base_lidar.*')),
)
