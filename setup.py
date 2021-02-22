from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()


setup(
    name='reachy_masks',
    version='1.0.0',
    description='Mask playground for Reachy robot',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/pollen-robotics/reachy_flyers',
    author='Pollen-Robotics',
    author_email='contact@pollen-robotics.com',
    packages=find_packages(exclude=['tests']),
    python_requires='>=3.5',
    install_requires=[
        'numpy',
        'zzlog',
    ],
)