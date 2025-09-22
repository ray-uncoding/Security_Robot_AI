from setuptools import find_packages
from setuptools import setup

setup(
    name='wheeltec_rrt_msg',
    version='0.0.0',
    packages=find_packages(
        include=('wheeltec_rrt_msg', 'wheeltec_rrt_msg.*')),
)
