import rclpy
from rclpy.node import Node

# Global Variables to be imported
GlobalNode = None
GlobalLogger = None
Publisher = None
Subscriber = None
Service = None

# Start ROS Node and create global reference to log from any file
class CoordinatorNodeHandler(Node):
    def __init__(self):
        super().__init__('coordinator')

        global GlobalLogger
        GlobalLogger = self.get_logger()

        global Publisher
        Publisher = self.Publisher

        global Subscriber
        Subscriber = self.Subscriber

        global Service
        Subscriber = self.Service

    def Publisher(self, topic, msg, callback):
        return self.create_publisher(msg, topic, callback, 10) or None

    def Subscriber(self, topic, msg):
        return self.create_subscription(msg, topic)

    def Service(self, topic, msg, callback):
        return self.create_service(topic, msg, callback)

# Method to set the empty GlobalNode object
def initialise_ros2_node(args=None):
    global GlobalNode
    GlobalNode = CoordinatorNodeHandler()
