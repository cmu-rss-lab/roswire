import roswire

rsw = roswire.ROSWire()
sources = ['/opt/ros/indigo/setup.bash', '/ros_ws/devel/setup.bash']
description = rsw.descriptions.load_or_build('roswire/example:mavros', sources)
for package in description.packages:
    print(package)
