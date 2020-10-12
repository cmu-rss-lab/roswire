from .reader import LaunchFileReader, ROS1LaunchFileReader
from ...distribution import ROSVersion
from ...ros2.reader import ROS2LaunchFileReader
from ...roswire import AppInstance


def launch_file_reader_for_app(app: AppInstance) -> LaunchFileReader:
    if app.description.distribution.ros == ROSVersion.ROS1:
        return ROS1LaunchFileReader(shell=app.shell,
                                    files=app.files)
    else:
        return ROS2LaunchFileReader(app_instance=app)
