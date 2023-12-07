import time
import unittest
import rclpy

from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import Int16

from test_processing_node import get_config_file_path
from processing_node.processing_node import ProcessingNode


test_parameter_dict = {'test int': int}

def sample_function(main_value, parameters: test_parameter_dict):
    return {"float parameter": main_value["Motor Current 1"][0]*5, "int parameter": parameters['test int']}

class TestProcessingNodeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_processing_node_integration')

        pub = self.node.create_publisher(Float32, 'sensor_data', 10)

        self.received_ints = []
        self.received_floats = []

        sub_float = self.node.create_subscription(Float32,
            'ExampleFloat',
            lambda msg: self.received_floats.append(msg.data),
            10
        )
        sub_int = self.node.create_subscription(Int16,
            'ExampleInt',
            lambda msg: self.received_ints.append(msg.data),
            10
        )

        msg = Float32()
        msg.data = 20.0

        second_executor = SingleThreadedExecutor()
        processsing_node = ProcessingNode(sample_function, get_config_file_path('config.yaml'))
        processsing_node.set_parameters([rclpy.parameter.Parameter('test int', value=8)])
        second_executor.add_node(processsing_node)
        second_executor.add_node(self.node)

        end_time = time.time() + 10
        while time.time() < end_time:

            pub.publish(msg)

            second_executor.spin_once()

            if len(self.received_ints)*len(self.received_floats)>1:
                break

        processsing_node.destroy_node()

    def tearDown(self):
        self.node.destroy_node()

    def test_processing_node_float_publisher(self):
        """
        This tests wether or not the ProcessingNode publishes on the topic "ExampleFloat" given in the
        test_config
        """
        assert len(self.received_floats)>0, 'Did not receive messages from topic "ExampleFloat"'

    def test_processing_node_int_publisher(self):
        """
        This tests wether or not the ProcessingNode publishes on the topic "ExampleInt" given in the
        test_config
        """
        assert len(self.received_ints)>0, 'Did not receive messages from topic "ExampleInt"'

    def test_processing_node_calculate_output(self):
        """
        This tests wether or not the ProcessingNode correctly uses the function specified in the
        construction call to calculate an output from the given Input on topic "Motor Current 1"
        and sends this output on topic "ExampleFloat"
        """
        assert self.received_floats[0] == 100.0, 'Calculating output from input using the specified function does not work'

    def test_processing_node_setting_parameters(self):
        """
        This tests wether or not the parameter "test int" demanded by the function specified in
        the construction call can be modified. The function returns this parameter and the
        ProcessingNode is supposed to publish it on the topic "ExampleInt"
        """
        assert self.received_ints[0] == 8, 'Parameter setting does not work'
