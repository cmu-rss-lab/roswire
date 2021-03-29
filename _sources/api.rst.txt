.. -*-restructuredtext-*-

API Reference
=============

This section of the documentation provides a reference to the public API for
the roswire library.

.. py:module:: roswire
.. autoclass:: ROSWire
   :members:


System
------

The :class:`System` class is used to provide access to a ROS application that
has been launched by :meth:`ROSWire.launch`. This class is the main entry
point for users wishing to interact with an ROS application.
The :class:`System` class facilitates interaction with the ROS application
through a number of loosely-coupled proxies that are represented as
attributes. For example, :attr:`System.shell` exposes a proxy for interacting
with a :code:`bash` shell inside the application container.

.. autoclass:: System()
  :members:


ROS
---

.. py:module:: roswire.proxy
.. autoclass:: ROSCore()
  :members:


.. autoclass:: SystemState()
   :members:


ROS Launch
----------

.. autoclass:: ROSLaunchManager()
   :members:


Nodes
-----

.. autoclass:: NodeManager()
  :show-inheritance:
  :members:
  :inherited-members:

  .. automethod:: __getitem__
  .. automethod:: __delitem__
  .. automethod:: __iter__
  .. automethod:: __len__

.. autoclass:: Node()
   :members:


Services
--------

.. autoclass:: ServiceManager()
  :show-inheritance:
  :members:
  :inherited-members:

  .. automethod:: __getitem__
  .. automethod:: __delitem__
  .. automethod:: __iter__
  .. automethod:: __len__

.. autoclass:: Service()
   :members:


Descriptions
------------

.. py:module:: roswire.description
.. autoclass:: SystemDescriptionManager()
  :members:

.. autoclass:: SystemDescription()
  :members:

.. py:module:: roswire.definitions
.. autoclass:: PackageDatabase()

.. autoclass:: TypeDatabase()

.. autoclass:: FormatDatabase()


Messages
--------
.. autoclass:: Message()
   :members:

.. autoclass:: MsgFormat()
   :members:

.. autoclass:: Field()
   :members:

.. autoclass:: Constant()
   :members:

.. autoclass:: SrvFormat()
   :members:

.. autoclass:: ActionFormat()
   :members:


Exceptions
----------

.. py:module:: roswire.exceptions

.. autoclass:: ROSWireException
.. autoclass:: CatkinException

.. autoclass:: FailedToParseLaunchFile
.. autoclass:: EnvNotFoundError
.. autoclass:: SubstitutionError
.. autoclass:: CatkinBuildFailed
.. autoclass:: CatkinCleanFailed
.. autoclass:: PlayerNotStarted
.. autoclass:: PlayerAlreadyStarted
.. autoclass:: PlayerAlreadyStopped
.. autoclass:: PlayerFailure
.. autoclass:: PlayerTimeout
.. autoclass:: RecorderAlreadyStarted
.. autoclass:: RecorderNotStarted
.. autoclass:: RecorderAlreadyStopped
.. autoclass:: ParsingError
.. autoclass:: NodeNotFoundError
.. autoclass:: ServiceNotFoundError
.. autoclass:: ParameterNotFoundError
