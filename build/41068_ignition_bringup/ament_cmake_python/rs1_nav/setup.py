from setuptools import find_packages
from setuptools import setup

setup(
    name='rs1_nav',
    version='1.0.5',
    packages=find_packages(
        include=('rs1_nav', 'rs1_nav.*')),
)
