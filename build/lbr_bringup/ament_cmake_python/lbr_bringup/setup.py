from setuptools import find_packages
from setuptools import setup

setup(
    name='lbr_bringup',
    version='2.2.0',
    packages=find_packages(
        include=('lbr_bringup', 'lbr_bringup.*')),
)
