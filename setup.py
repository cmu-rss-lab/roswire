import os
from glob import glob
from setuptools import setup, find_packages


path = os.path.join(os.path.dirname(__file__), 'src/rozzy/version.py')
with open(path, 'r') as f:
    exec(f.read())


setup(
    name='rozzy',
    version=__version__,
    python_requires='>=3.5',
    description='An API for testing and mutating ROS systems.',
    author='Chris Timperley',
    author_email='ctimperley@cmu.edu',
    url='https://github.com/squaresLab/rozzy',
    license='mit',
    install_requires=[
        'bugzoo>=2.1.22',
        'attrs>=17.2.0'
    ],
    setup_requires=[
        'pytest-runner'
    ],
    tests_require=[
        'pytest'
    ],
    include_package_data=True,
    packages=find_packages('src'),
    package_dir={'': 'src'},
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    keywords=['ros', 'docker', 'testing', 'mutation', 'bug', 'repair'],
    classifiers=[
        'Natural Language :: English',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7'
    ],
)
