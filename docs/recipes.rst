.. -*-restructuredtext-*-

Recipes
=======

This section of the documentation provides several recipes for using ROSWire.

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


Produce a description of a given ROS package
--------------------------------------------

.. code:: python

  import roswire

  rsw = roswire.ROSWire()
  description = rsw.descriptions.load_or_build('roswire/example:mavros')
  package = description.packages['tf2_msgs']

  print("PACKAGE DETAILS")
  print('-' * 80)
  print(f"Name: {package.name}")
  print(f"Path: {package.path}")
  print(f"Messages: {', '.join(m.name for m in package.messages)}")
  print(f"Services: {', '.join(s.name for s in package.services)}")
  print(f"Actions: {', '.join(a.name for a in package.actions)}")


Running the code snippet above produces the following output:

.. code:: shell

  PACKAGE DETAILS
  --------------------------------------------------------------------------------
  Name: tf2_msgs
  Path: /opt/ros/indigo/share/bond
  Messages: TF2Error, TFMessage
  Services: FrameGraph
  Actions: LookupTransform
