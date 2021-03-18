import os
from glob import glob
from setuptools import setup, find_packages


path = os.path.join(os.path.dirname(__file__), "src/roswire/version.py")
with open(path, "r") as f:
    exec(f.read())


setup(version=__version__,
      package_data={'': ['src/roswire/resources/launch_esxtractor._py']},
      inlcude_package_data=True)
