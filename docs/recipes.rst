.. -*-restructuredtext-*-

Recipes
=======

To appear.

.. note::

  If you would like to see a recipe here, then go ahead and request a recipe by
  submitting an issue to the GitHub issue tracker: https://github.com/ChrisTimperley/roswire/issues


List all ROS packages within a Docker image
-------------------------------------------

The following example prints a list of the names of all of the ROS packages
that appear in a given Docker image.

.. code:: python

  import roswire

  rsw = roswire.ROSWire()
  description = rsw.descriptions.load_or_build('roswire/example:mavros')
  for package in description.packages:
    print(package)

Running the example above produces the following output:

.. code:: shell

  catkin
  genmsg
  gencpp
  genlisp
  genpy
  cmake_modules
  test_find_tinyxml
  class_loader
  cpp_common
  mavlink
  libmavconn
  ...
