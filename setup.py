import re
import os
import setuptools
import bmp_280

with open("README.md", "r") as fh:
    long_description = fh.read()

def read_file(path):
    with open(os.path.join(os.path.dirname(__file__), path)) as fp:
        return fp.read()

def _get_version_match(content):
    # Search for lines of the form: # __version__ = 'ver'
    regex = r"^__version__ = ['\"]([^'\"]*)['\"]"
    version_match = re.search(regex, content, re.M)
    if version_match:
        return version_match.group(1)
    raise RuntimeError("Unable to find version string.")

def get_version(path):
    return _get_version_match(read_file(path))

setuptools.setup(
    name="bmp_280",
    version=get_version(os.path.join('bmp_280', '__init__.py')),
    scripts=['bin/run-bmp280'],
    author="Feyzi Kesim",
    author_email="feyzikesim@gmail.com",
    description="Python3 I2C Driver & Application for BMP280 Pressure Sensor",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/feyzikesim/bmp280",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
