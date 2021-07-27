import os
import setuptools


path = os.path.join(os.path.dirname(__file__), "src/roswire/version.py")
with open(path, "r") as f:
    exec(f.read())


setuptools.setup(version=__version__)
