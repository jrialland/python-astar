
#!/usr/bin/env python
from setuptools import setup, find_packages

setup(
    name='astar',
    version='0.9',
    author='Julien Rialland',
    author_email='julien.rialland@gmail.com',
    url='http://github.com/jrialland/python-astar',
    description='generic a-star implementation',
    provides=['astar'],
    long_description='generic a-star implementation',
    zip_safe=True,
    license='BSD',
    package_dir={'':'src'},
    packages = find_packages("src", exclude=["test"]),
    include_package_data=True,
    classifiers=[
        'Intended Audience :: Developers',
        'Operating System :: OS Independent',
        'Topic :: Software Development',
        'Programming Language :: Python',
    ]
)
