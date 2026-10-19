"""Keep the independent converter Python ABI separate from the ROS Conda ABI."""
import os


def converter_environment():
    removed = {'PYTHONHOME', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
               'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CONDA_PREFIX',
               'CONDA_DEFAULT_ENV', 'QT_QPA_PLATFORM_PLUGIN_PATH'}
    return {name: value for name, value in os.environ.items() if name not in removed}
