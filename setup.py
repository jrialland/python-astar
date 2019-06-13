#!/usr/bin/env python
import sys
from distutils.core import setup, Extension
from glob import glob

import astar

setup(
    name='astar',
    version=astar.__version__,
    author=astar.__author__,
    author_email=astar.__email__,
    maintainer=astar.__maintainer__,
    url='http://github.com/jrialland/python-astar',
    description='A-* algorithm implementation',
    packages=['astar'],
    provides=['astar'],
    long_description='Provides path finding using A-*',
    license=astar.__license__,
    package_dir={'astar': 'astar'},
    ext_modules = [Extension('astar_native', sources = glob('astar_native_c_code/*.c'))],
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
