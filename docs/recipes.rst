.. -*-restructuredtext-*-

Recipes
=======

To appear.

.. note::

  If you would like to see a recipe here, then go ahead and request a recipe by
  submitting an issue to the GitHub issue tracker: https://github.com/ChrisTimperley/roswire/issues


List all ROS packages within a Docker image
-------------------------------------------

.. code:: python

  import roswire

  rsw = roswire.ROSWire()
  description = rsw.descriptions.load_or_build('christimperley/turtlebot')
  for package in description.packages:
    print(package)

  # 
