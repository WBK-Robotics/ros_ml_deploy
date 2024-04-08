import time
import unittest
import rclpy

from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import String

from geometry_msgs.msg import Vector3

from test_processing_node import get_config_file_path
from processing_node.processing_node import ProcessingNode



class SampleProcessor:
    def __init__(self):
        self.parameters = {'test int': int}

    def get_parameters(self):
        return self.parameters
    
    def set_parameters(self, parameters: dict):
        for key in parameters.keys():
            if key in self.parameters.keys():
                self.parameters[key] = parameters[key]
            else:
                raise ValueError("The parameter {} is not supported by this processor".format(key))
    
    def execute(self, main_value):
        try:
            return {"float parameter": 5*float(main_value['Motor Current 1'][0]), 
                "string parameter": 'Test_String', 
                "int parameter": self.parameters['test int'],
                "vector parameter x": 1.0,
                "vector parameter y": 1.0,
                "No Field Parameter": main_value['No Field Parameter'][0]
            }
        except:
            return None

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
        pub_fake = self.node.create_publisher(Float32, 'second_data_fake', 10)
        pub_third = self.node.create_publisher(Float32, 'third_data', 10)

        self.received_ints = []
        self.received_floats = []
        self.received_strs = []
        self.received_vectors = []
        self.received_float_messages = []

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

        sub_str = self.node.create_subscription(String,
            'ExampleString',
            lambda msg: self.received_strs.append(msg.data),
            10
        )

        sub_vector = self.node.create_subscription(Vector3,
            'ExampleVector',
            lambda msg: self.received_vectors.append(1),
            10
        )

        sub_float_full = self.node.create_subscription(Float32,
            'ExampleNoField',
            lambda msg: self.received_float_messages.append(msg),
            10
        )

        msg = Float32()

        msg.data = 20.0
        pub.publish(msg)
        pub_fake.publish(msg)
        pub_third.publish(msg)

        second_executor = SingleThreadedExecutor()
        sample_processor = SampleProcessor()
        processsing_node = ProcessingNode(sample_processor, get_config_file_path('integration_config.yaml'))
        processsing_node.set_parameters([rclpy.parameter.Parameter('test int', value=8)])
        second_executor.add_node(processsing_node)
        second_executor.add_node(self.node)

        end_time = time.time() + 20
        while time.time() < end_time:

            pub.publish(msg)
            pub_fake.publish(msg)
            pub_third.publish(msg)

            second_executor.spin_once()

            if len(self.received_ints)*len(self.received_floats)*len(self.received_strs)*len(self.received_float_messages)*len(self.received_vectors)>0:
                break

        self.fake_pub_subscriber_count = pub_fake.get_subscription_count()
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
    
    def test_processing_node_vector_publisher(self):
        """
        This tests wether or not the ProcessingNode publishes on the topic "ExampleVector" given in the
        test_config
        """
        assert len(self.received_vectors)>0, 'Did not receive messages from topic "ExampleVector"'
    
    def test_subscriber_setup(self):
        """
        This tests wether or not the ProcessingNode subscribes to the correct topic "sensor_data" 
        and not "sensor_data_fake"
        """
        assert self.fake_pub_subscriber_count == 0, 'Subscribed to the wrong topic!'

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
    
    def test_multiple_parameters_same_message(self):
        """
        This tests wether multiple parameters on the same message lead to multiple messages being sent
        or not
        """
        assert len(self.received_vectors) == len(self.received_strs), 'Number of messages received on topic "ExampleVector" differs from the number of messages received on topic "ExampleString"'

    def test_no_field_messages(self):
        """
        This tests wether messages with no field specified in the config work for both subscribing and publishing
        """
        assert len(self.received_float_messages) > 0, "Did not receive messsages from topic ExampleNoField"
        assert self.received_float_messages[0].data == 20.0, "Message from topic ExampleNoField carries incorrect data"

    def test_string(self):
        assert self.received_strs[0] == "Test_String", "String does not work"

