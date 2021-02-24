"""Launch in a component container."""

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros import get_default_launch_description
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch_ros.actions import Node
import datetime
import string
import os

# Set to 0 to disable multiple log files (and log rotation).
LOG_WRITE_LIMIT = 10000

# Set to 0 to disable log rotation
LOG_ROTATION_LIMIT = 100

# True if we want to log the time stamp, False otherwise
TIMESTAMP = True


def timestamp():
    return "[{0:.6f}]".format(datetime.datetime.now().timestamp())


def launch_print(text):
    print("[INFO] [launch.py] " + text)


class Logger:
    # Class used for logging output to file

    # Initializes the logger
    def __init__(self):
        # ISO8601 standard time with zero offset.
        self.date = datetime.datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        self.index = 1
        self.prune = 1
        self.line = 0
        filename = self.current_filename()
        directory = os.path.dirname(filename)
        if not os.path.exists(directory):
            os.makedirs(directory)

        launch_print("Logging to file: " + filename)
        self.file = open(filename, "wb")
        self.update_symlink(filename)

    def get_indexed_filename(self, index):
        return os.getcwd() + '/log/' + self.date + ("_%03d.log" % index)

    def current_filename(self):
        if LOG_WRITE_LIMIT > 0:
            return self.get_indexed_filename(self.index)
        return os.getcwd() + '/log/' + self.date + ".log"

    def update_symlink(self, filename):
        tmpfile = os.getcwd() + '/log/tmp'
        os.symlink(os.path.basename(filename), tmpfile)
        os.replace(tmpfile, os.getcwd() + '/log/latest.log')

    def increment(self):
        self.line = 0
        self.index += 1
        self.file.close()
        filename = self.current_filename()
        launch_print("Incrementing log file to: " + filename)
        self.file = open(filename, "wb")
        self.update_symlink(filename)
        if LOG_ROTATION_LIMIT > 0 and self.index > LOG_ROTATION_LIMIT:
            filename = self.get_indexed_filename(self.prune)
            launch_print("Pruning oldest log file: " + filename)
            os.remove(filename)
            self.prune += 1

    def write(self, text):
        if TIMESTAMP:
            self.file.write(timestamp().encode('utf-8'))
        self.file.write(text)
        self.file.flush()
        self.line += max(1, text.count(b"\n"))
        if LOG_WRITE_LIMIT > 0 and self.line > LOG_WRITE_LIMIT:
            self.increment()


def generate_launch_description():
    """Setup Logging"""
    log = Logger()

    """Load Configuration File"""
    param_file = os.getcwd() + '/params.yaml'
    # this prevents everything crashing if you deleted the config.yaml
    node_remaps = []
    if os.path.isfile(param_file):
        node_remaps += [('__params', param_file)]
        launch_print("Using parameters file: " + param_file)
    else:
        launch_print("Parameters file not found: " + param_file)

    """Generate launch description with multiple components."""
    ld = get_default_launch_description()

    ld.add_action(launch.actions.DeclareLaunchArgument(
        'gdb',
        default_value='false',
        description="Launch using GDB."
    ))
    ld.add_action(launch.actions.SetLaunchConfiguration(
        name='launch-prefix',
        value='gdbserver :2000',
        condition=IfCondition(LaunchConfiguration('gdb'))
    ))

    ld.add_action(ComposableNodeContainer(
        node_name='example_container',
        node_namespace=[],
        package='rclcpp_components',
        node_executable='component_container',

        composable_node_descriptions=[

                ComposableNode(
                    package='example',
                    node_plugin='ExampleConfig',
                    node_name='exampleConfig',
                    remappings=node_remaps),

                ComposableNode(
                    package='example',
                    node_plugin='ExamplePub',
                    node_name='examplePub',
                    remappings=node_remaps),
                ComposableNode(
                    package='example',
                    node_plugin='ExampleSub',
                    node_name='exampleSub',
                    remappings=node_remaps),

                ComposableNode(
                    package='example',
                    node_plugin='ExampleTime',
                    node_name='exampleTime',
                    remappings=node_remaps),

                ComposableNode(
                    package='example',
                    node_plugin='ExampleLog',
                    node_name='exampleLog',
                    remappings=node_remaps),

                ComposableNode(
                    package='example',
                    node_plugin='ExampleTF',
                    node_name='example_tf_node',
                    remappings=node_remaps),

                ComposableNode(
                    package='example',
                    node_plugin='ExampleTF_filter',
                    node_name='example_tf_filter_node',
                    remappings=node_remaps),
        ],
        output='screen',
        # arguments=[('__log_level:=debug')]
    ))
    ld.add_action(
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                on_stdout=lambda info: log.write(info.text),
                on_stderr=lambda info: log.write(info.text),
            ))
    )
    # broadcast static tf.
    # Example of reading this trasnformation is in example_tf.cc
    # translation: 2m in x axis, 0m in y axis, 2m in z axis
    # rotation: 0 degrees in x axis, 0 degrees in y axis, 0 degrees in z axis
    ld.add_action(Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        arguments=['2', '0', '2', '0', '0', '0', 'parent', 'child'],
        output='both',
    ))
    return ld
