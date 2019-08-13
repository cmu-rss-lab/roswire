.. -*-restructuredtext-*-

roswire
=======


ROSWire is a Python library for performing static and dynamic analysis of
containerised `Robot Operating System (ROS) <https://ros.org>`_
applications.
Given a ROS application in the form of a `Docker <https://docker.org>`_
image, ROSWire provides an interface for statically querying the application
(e.g., by automatically discovering its types, packages, messages, service,
actions, etc.), as well as an interface for dynamically generating and
interacting with instances of that application in the form of Docker
containers (e.g., service calls, bag recording, topic publishing and
subscribing, catkin builds, etc.).


Features
--------

* **Does not require ROS to be installed on your machine.**
* **Supports most ROS distributions out of the box (e.g., Groovy, Indigo, Kinetic, Melodic).**
* **Package Discovery:** finds all ROS packages within a Docker image.
* **Definition Discovery:** finds and parses all message, service and
  action formats into readable data structures.
* **Message Serialisation:** converts ROS messages from YAML or binary
  to readable data structures and vice versa.
* **Bag Manipulation:** efficiently parses
  `rosbag <http://wiki.ros.org/rosbag>`_ files, which can then be inspected,
  manipulated, and saved to disk.
* **Bag Playback:** safely replay bag files inside containers.


Installation
------------

To avoid interfering with the rest of your system (i.e., to avoid Python's
equivalent of DLL hell), we strongly recommend that
ROSWire is installed within a
`virtualenv <https://virtualenv.pypa.io/en/latest/>`_ or
`pipenv <https://pipenv.readthedocs.io/en/latest/>`_ (pipenv is preferred).

From within the virtual environment (i.e., the `virtualenv` or `pipenv`),
the latest stable release of ROSWire on `PyPI <https://pypi.org>`_
can be installed via:

.. code:: shell

   (roswire) $ pip install roswire

ROSWire can also be installed from source:

.. code:: shell

   $ git clone git@github.com:ChrisTimperley/roswire roswire
   $ cd roswire
   $ pipenv shell
   (roswire) $ pip install .


Getting Started
---------------


To obtain a description of all of the packages contained within a ROS
application, along with a description of their associated message,
service and action types:


.. code:: python

   image = 'myrosapp'  # name of the Docker image for the ROS app

   # create a new roswire session
   roswire = ROSWire()

   # we can obtain a static description of the application
   #
   # * loads an existing description if one has already been generated and
   #   saved to disk for the given ROS application
   # * otherwise it builds a description for the application and caches that
   #   description to disk (unless told not).
   # * descriptions in the cache are indexed by the SHA256 ID of the Docker
   #   image. If the image changes or a different tag is used for the same
   #   image, ROSWire is smart enough to load an existing description or build
   #   a new one accordingly.
   #
   description = roswire.descriptions.load_or_build(image)


API Reference
-------------


.. py:module:: roswire
.. autoclass:: ROSWire
  :members:

.. py:module:: roswire.description
.. autoclass:: SystemDescriptionManager()
  :members:

.. autoclass:: SystemDescription()
  :members:



Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Contents:
