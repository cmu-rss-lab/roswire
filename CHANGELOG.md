# 1.0.1 (????-??-??)

* Bug fix: Updated `proxy.substitution` module to use DockerBlade API
* Containers will now raise a `SourceNotFoundError` if a given source does
  not exist inside the container (#338).


# 1.0.0 (2020-31-03)

* Added `ports` keyword argument to `launch` method for `ROSWire`, allowing
  users to specify an optional container-host port mapping.
* Added required `sources` positional argument to `launch` method for `ROSWire`
  and most methods for `DescriptionManager` to allow user to specify the setup
  files that should be used by the application.
* Used `dockerblade` as a dependency to reduce the size of the code base,
  ease maintenance, and improve cohesion.


# 0.0.5 (2019-08-28)

* Created initial CHANGELOG.
