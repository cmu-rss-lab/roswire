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

.. literalinclude:: recipes/list_all_ros_packages.py
   :language: python

Running the example above produces the following output:

.. literalinclude:: recipes/list_all_ros_packages.out


Produce a description of a given ROS package
--------------------------------------------

.. literalinclude:: recipes/describe_ros_package.py
   :language: python

Running the code snippet above produces the following output:

.. literalinclude:: recipes/describe_ros_package.out


Call a ROS service
------------------

.. literalinclude:: recipes/service_call.py
   :language: python


Apply a source code patch and rebuild the application
-----------------------------------------------------


.. literalinclude:: recipes/patch_and_rebuild.py
   :language: python


Below are the contents of :code:`example.diff`.

.. literalinclude:: recipes/example.diff
   :language: diff
