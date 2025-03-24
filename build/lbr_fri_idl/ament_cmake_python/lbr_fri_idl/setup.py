from setuptools import find_packages
from setuptools import setup

setup(
    name='lbr_fri_idl',
    version='1.5.0',
    packages=find_packages(
        include=('lbr_fri_idl', 'lbr_fri_idl.*')),
)
