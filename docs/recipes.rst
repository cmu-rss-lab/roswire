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


Apply a source code patch and rebuild the application
-----------------------------------------------------


.. code:: python

   import roswire

   rsw = roswire.ROSWire()

   with open('example.diff') as f:
      diff = f.read()

   with rsw.launch('roswire/example:mavros') as system:
      print("applying patch...")
      context = '/ros_ws/src/mavros/mavros/src/mavros_node.cpp'
      system.files.patch(context, diff)
      print("patch applied")

      # rebuild via catkin tools
      print("rebuilding...")
      dir_workspace = '/ros_ws'
      system.catkin(dir_workspace).build()
      print("rebuilt")


Below are the contents of :code:`example.diff`.

.. code:: diff

   --- mavros_node.cpp	2019-09-09 23:22:23.000000000 +0000
   +++ mavros_node.cpp	2019-09-10 00:23:23.952098138 +0000
   @@ -20,6 +20,6 @@
    	mavros::MavRos mavros;
    	mavros.spin();
    
   -	return 0;
   +	return 1;
    }
    
   
