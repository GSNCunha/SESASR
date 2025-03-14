from setuptools import find_packages
from setuptools import setup

setup(
    name='landmark_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('landmark_msgs', 'landmark_msgs.*')),
)
