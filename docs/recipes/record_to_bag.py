import roswire

rsw = roswire.ROSWire()

with rsw.launch('roswire/example:mavros') as system:
    pass
