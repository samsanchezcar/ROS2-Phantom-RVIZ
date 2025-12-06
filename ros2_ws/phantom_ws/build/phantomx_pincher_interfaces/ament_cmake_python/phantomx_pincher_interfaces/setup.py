from setuptools import find_packages
from setuptools import setup

setup(
    name='phantomx_pincher_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('phantomx_pincher_interfaces', 'phantomx_pincher_interfaces.*')),
)
