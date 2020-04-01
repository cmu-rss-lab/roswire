import roswire

rsw = roswire.ROSWire()

with open('example.diff') as f:
    diff = f.read()

# we use 'launch' to create a temporary container for the application
# when the context is closed, either by reaching the end of the with
# block or by abruptly encountering an exception, the container will be
# automatically destroyed.
sources = ['/opt/ros/indigo/setup.bash', '/ros_ws/devel/setup.bash']
with rsw.launch('roswire/example:mavros', sources) as system:
    print("applying patch...")
    context = '/ros_ws/src/mavros/mavros/src/mavros_node.cpp'
    system.files.patch(context, diff)
    print("patch applied")
    
    # rebuild via catkin tools
    print("rebuilding...")
    dir_workspace = '/ros_ws'
    catkin = system.catkin(dir_workspace)
    catkin.build()
    print("rebuilt")
