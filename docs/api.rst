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

.. autoclass:: System
  :members:


Descriptions
------------

.. py:module:: roswire.description
.. autoclass:: SystemDescriptionManager()
  :members:

.. autoclass:: SystemDescription()
  :members:


Containers
----------

.. py:module:: roswire

Internally, roswire uses the following classes to facilitate its interactions
with Docker. The majority of users should not need to interact with these
classes directly; instead, they should use interact with :class:`System`
instances that are generated using :meth:`ROSWire.launch`.

.. py:module:: roswire.proxy.container
.. autoclass:: ContainerProxyManager()
  :members:

.. autoclass:: ContainerProxy()
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
