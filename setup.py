#!/usr/bin/env python

from distutils.cmd import Command
from distutils.core import setup, Extension
from glob import glob


class TestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        import sys, subprocess
        raise SystemExit(
            subprocess.call([sys.executable,
                             '-m',
                             'tests']))

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
    cmdclass = {'test' : TestCommand},
    ext_modules = [
        Extension(
            'astar_native',
            sources = [f for f in glob('astar_native_c_code/*.c') if not 'main' in f],
            extra_compile_args = ['-std=c99']
        )
    ],
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
