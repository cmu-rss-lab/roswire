import os
from glob import glob
from setuptools import setup, find_packages


path = os.path.join(os.path.dirname(__file__), 'src/roswire/version.py')
with open(path, 'r') as f:
    exec(f.read())


setup(
    version=__version__,
    python_requires='>=3.6',
    install_requires=[
        'attrs>=17.2.0',
        'typing-extensions>=3.7.2',
        'ruamel.yaml>=0.15.89',
        'toposort~=1.5',
        'docker~=3.7.2',
        'psutil~=5.6.2',
        'pyyaml~=5.1'
    ],
    setup_requires=[
        'pytest-runner'
    ],
    tests_require=[
        'pytest'
    ],
    packages=find_packages('src'),
    package_dir={'': 'src'},
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    test_suites='tests'
)
