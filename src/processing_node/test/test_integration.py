import time
import unittest
import rclpy

from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import Int16


from processing_node.processing_node import ProcessingNode
from test_processing_node import get_config_file_path

test_parameter_dict = {'test int': int}

def weird_function(main_value, parameters: test_parameter_dict):
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

    def tearDown(self):
        self.node.destroy_node()
    
    def test_processing_node_output(self):
        
        pub = self.node.create_publisher(Float32, 'sensor_data', 10)

        received_ints = []
        received_floats = []

        sub_float = self.node.create_subscription(Float32, 'ExampleFloat', lambda msg: received_floats.append(msg.data), 10)
        sub_int = self.node.create_subscription(Int16, 'ExampleInt', lambda msg: received_ints.append(msg.data), 10)

        msg = Float32()
        msg.data = 20.0

        pub.publish(msg)

        self.second_executor = SingleThreadedExecutor()
        self.processsing_node = ProcessingNode(weird_function, get_config_file_path('config.yaml'))
        self.processsing_node.set_parameters([rclpy.parameter.Parameter('test int', value=8)])
        self.second_executor.add_node(self.processsing_node)
        self.second_executor.add_node(self.node)

        end_time = time.time() + 30
        while time.time() < end_time:
            
            pub.publish(msg)

            self.second_executor.spin_once()

            if len(received_floats) > 2:
                break
        assert received_floats[0] == msg.data*5, f'Calculating output from input using the specified function does not work'
        assert received_ints[0] == 8, f'Parameter setting does not work'

        self.processsing_node.destroy_node()
        
        

