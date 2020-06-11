# 1.4.0 (XXXX-XX-XX)

* Added `restrict_to_topics` argument to `record` method of `ROSCore`, allowing
  users to restrict bag recording to specific topics by supplying a regular
  expression in the same manner as the `rosbag` command line tool.
* Improved documentation for ROSCore, its associated parts, and messages.
* Added `state` property to `ROSCore` along with associated
  `SystemState` and `SystemStateProbe` classes.
* Added `command` property to `Parameter`, responsible for recording the
  command, if any, that was used to obtain the concrete value for a given
  parameter.
* Added ability to provide node-specific name remappings when using `roslaunch`
  through the introduction of the `remappings` argument to the `launch`
  method.
* Fixed mishandling of complex parameters when reading rosparam tags.
* Added `to_xml_string` and `to_xml_file` to `LaunchConfig`.


# 1.3.0 (2020-06-02)

* Added ability to specify additional environment variables when launching
  systems
* Allowed mypy to discover type hints via PEP 561
* Added missing type annotations
* Added ROSLaunchController for dynamically inspecting and controlling
  roslaunch processes


# 1.2.1 (2020-05-27)

* Bug fix: Avoid abrupt container termination due to use of default
  entrypoint by using `/bin/sh` as an entrypoint.


# 1.2.0 (2020-05-27)

* Moved methods in `substitution` module into `ArgumentResolver` class.
* Added handling of executables and resources for `$(find ...)` commands.
* Fixed handling of namespaces for `node` elements in XML launch files.
* Fixed handling of `$(optenv ...)` tags in XML launch files.
* Fixed handling of `launch-prefix` for `node` elements in XML launch files.
* Fixed bad type annotation in `proxy.launch.substitution` that broke Python
  3.6 compatibility.
* Moved all logging from Python's built-in logging library to loguru.
* Added `to_xml_tree` method to `LaunchConfig`.
* Added `PackageNotFound` and `LaunchFileNotFound` exception.
* Added `roslaunch` property to `ROSCore`, which exposes a `ROSLaunchManager`.
  The manager provides various `roslaunch`-related functionality including
  locating, generating, parsing, flattening, and launching launch files.
* Removed `launch` method from `ROSCore`. Replaced with `roslaunch`.


# 1.1.0 (2020-04-23)

* Bug fix: Updated `proxy.substitution` module to use DockerBlade API
* Containers will now raise a `SourceNotFoundError` if a given source does
  not exist inside the container (#338).
* Container spawning no longer relies on the creation of a temporary file
  for storing environment variables inside the container. This allows
  ROSWire to work with more Docker images without the need for manual
  changes.
* Bag recorder now implements the `exclude_topics` parameter rather than
  ignoring it.
* Renamed `CatkinProxy`, `CatkinMakeProxy`, and `CatkinToolsProxy` to
  `CatkinInterface`, `CatkinMake`, and `CatkinTools`, respectively.
* Renamed `ROSProxy` to `ROSCore`.
* Renamed `BagPlayerProxy` and `BagRecorderProxy` to `BagPlayer` and
  `BagRecorder`, respectively.
* Renamed `ContainerProxy` and `ContainerManagerProxy` to `Container` and
  `ContainerManager`.
* Renamed `NodeProxy` and `NodeManagerProxy` to `Node` and `NodeManager`.
* Renamed `ParameterServerProxy` to `ParameterServer`.
* Renamed `ServiceProxy` and `ServiceProxyManager` to `Service` and
  `ServiceManager`.
* Removed `client_docker` property from `ROSWire`.


# 1.0.0 (2020-03-31)

* Added `ports` keyword argument to `launch` method for `ROSWire`, allowing
  users to specify an optional container-host port mapping.
* Added required `sources` positional argument to `launch` method for `ROSWire`
  and most methods for `DescriptionManager` to allow user to specify the setup
  files that should be used by the application.
* Used `dockerblade` as a dependency to reduce the size of the code base,
  ease maintenance, and improve cohesion.


# 0.0.5 (2019-08-28)

* Created initial CHANGELOG.
