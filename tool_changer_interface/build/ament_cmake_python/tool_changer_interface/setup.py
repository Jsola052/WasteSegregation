from setuptools import find_packages
from setuptools import setup

setup(
    name='tool_changer_interface',
    version='0.0.0',
    packages=find_packages(
        include=('tool_changer_interface', 'tool_changer_interface.*')),
)
