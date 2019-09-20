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
