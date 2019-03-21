Rozzy
=====


Rozzy is a Python library for performing static and dynamic analysis of
containerised `Robot Operating System (ROS) <https://ros.org>`_
applications.
Given a ROS application in the form of a `Docker <https://docker.org>`_
image, Rozzy provides an interface for statically querying the application
(e.g., by automatically discovering its types, packages, messages, service,
actions, etc.), as well as an interface for dynamically generating and
interacting with instances of that application in the form of Docker
containers (e.g., service calls, bag recording, topic publishing and
subscribing, catkin builds, etc.).


Installation
------------

To avoid interfering with the rest of your system (i.e., to avoid Python's
equivalent of DLL hell), we strongly recommend that
Rozzy is installed within a
`virtualenv <https://virtualenv.pypa.io/en/latest/>`_ or
`pipenv <https://pipenv.readthedocs.io/en/latest/>`_ (pipenv is preferred).

From within the virtual environment (i.e., the `virtualenv` or `pipenv`),
the latest stable release of Rozzy on `PyPI <https://pypi.org>`_
can be installed via:

.. code:: shell

   (rozzy) $ pip install rozzy

Rozzy can also be installed from source:

.. code:: shell

   $ git clone git@github.com:ChrisTimperley/rozzy rozzy
   $ cd rozzy
   $ pipenv shell
   (rozzy) $ pip install .


Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Contents:
