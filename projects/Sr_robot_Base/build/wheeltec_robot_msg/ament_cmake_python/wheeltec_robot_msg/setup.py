from setuptools import find_packages
from setuptools import setup

setup(
    name='wheeltec_robot_msg',
    version='0.0.0',
    packages=find_packages(
        include=('wheeltec_robot_msg', 'wheeltec_robot_msg.*')),
)
