from setuptools import find_packages
from setuptools import setup

setup(
    name='turn_on_wheeltec_robot',
    version='0.0.0',
    packages=find_packages(
        include=('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.*')),
)
