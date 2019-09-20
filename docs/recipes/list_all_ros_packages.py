import roswire

rsw = roswire.ROSWire()
description = rsw.descriptions.load_or_build('roswire/example:mavros')
for package in description.packages:
    print(package)
