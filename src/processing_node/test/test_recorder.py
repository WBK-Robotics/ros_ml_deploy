import rclpy
import unittest

from std_msgs.msg import Float32

class TestRecorderNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_recorder_node')
        pub = self.node.create_publisher(Float32, 'sensor_data', 10)