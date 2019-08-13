.. -*-restructuredtext-*-

API Reference
=============

This section of the documentation provides a reference to the public API for
the roswire library.

.. py:module:: roswire
.. autoclass:: ROSWire
  :members:


Managers
--------

.. py:module:: roswire.description
.. autoclass:: SystemDescriptionManager()
  :members:

.. autoclass:: SystemDescription()
  :members:

.. py:module:: roswire.proxy.container
.. autoclass:: ContainerProxyManager()
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
