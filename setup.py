#!/usr/bin/env python
import sys
from setuptools import setup, find_packages
import astar

setup(
    name='astar',
    version=astar.__version__,
    author=astar.__author__,
    author_email=astar.__email__,
    maintainer=astar.__maintainer__,
    url='http://github.com/jrialland/python-astar',
    description='A-* algorithm implementation',
    provides=['astar'],
    long_description='Provides path finding using A-*',
    zip_safe=True,
    license=astar.__license__,
    package_dir={'astar': 'astar'},
    packages=['astar'],
    include_package_data=True,
    keywords=['a-star', 'search', 'path finding'],
    platforms=['any'],
    classifiers=[
        'Intended Audience :: Developers',
        'Operating System :: OS Independent',
        'Topic :: Software Development',
        'Topic :: Scientific/Engineering :: Information Analysis',
        'Programming Language :: Python',
    ]
)
