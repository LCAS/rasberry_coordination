import rclpy
from rclpy.node import Node
from rclpy.impl.logging_severity import LoggingSeverity

# Global Variables to be imported
GlobalNode = None
GlobalLogger = None


# Start ROS Node and create global reference to log from any file
class CoordinatorNodeHandler(Node):
    def __init__(self):
        super().__init__('coordinator')
        #self.loop_rate = self.create_rate(1, self.get_clock())

        global GlobalLogger
        GlobalLogger = self.get_logger()

# Method to set the empty GlobalNode object
def initialise_ros2_node(args=None):
    global GlobalNode
    GlobalNode = CoordinatorNodeHandler()
