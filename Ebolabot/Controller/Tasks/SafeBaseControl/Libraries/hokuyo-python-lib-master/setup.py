# coding=utf-8
# !/usr/bin/env python

try:
    from setuptools import setup
except ImportError:
    print('No setuptools installed, use distutils')
    from distutils.core import setup

with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(
    name='hokuyo-python-lib',
    packages=[
        'hokuyo',
        'hokuyo.driver',
        'hokuyo.tools',
        'hokuyo.tests'
    ],
    package_dir={
        'hokuyo': 'src/hokuyo',
        'hokuyo.driver': 'src/hokuyo/driver',
        'hokuyo.tools': 'src/hokuyo/tools',
        'hokuyo.tests': 'src/hokuyo/tests'
    },
    install_requires=required,
    version='1.3',
    description='Hokuyo driver in python',
    author=u'Paweł Suder',
    author_email='pawel@suder.info',
    url='http://project-capo.github.io/',
    download_url='http://github.com/old-paoolo/hokuyo-python-lib',
    keywords=[
        'hokuyo'
    ],
    classifiers=[
        'Programming Language :: Python',
        'Development Status :: 4 - Beta',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'License :: Other/Proprietary License',
        'Operating System :: OS Independent',
    ],
    long_description='''\
'''
)
