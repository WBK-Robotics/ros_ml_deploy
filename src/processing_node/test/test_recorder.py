import rclpy
import unittest
import csv

from io import StringIO
from rclpy.executors import SingleThreadedExecutor
from processing_node.recorder_node import RecorderNode
from test_processing_node import get_config_file_path

from std_msgs.msg import Float32
from std_msgs.msg import String

class TestRecorderNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_recorder_node')
        pub_float = self.node.create_publisher(Float32, 'record_data_float', 10)
        pub_string = self.node.create_publisher(String, 'record_data_string', 10)

        self.outfile = StringIO()

        second_executor = SingleThreadedExecutor()
        recorder_node = RecorderNode(get_config_file_path('recorder_test_config.yaml'), self.outfile, number_of_input_points=2)

        second_executor.add_node(self.node)
        second_executor.add_node(recorder_node)

        msg_float = Float32()
        msg_float.data = 20.0

        msg_string = String()
        msg_string.data = "abcd"

        for i in range(1000):
            pub_float.publish(msg_float)
            pub_string.publish(msg_string)

            second_executor.spin_once()

            if recorder_node.recording_done.done():
                break

    def tearDown(self):
        self.node.destroy_node()

    def test_file_output(self):
        self.outfile.seek(0)
        csv_reader = csv.DictReader(self.outfile)

        assert csv_reader.fieldnames == ['Param 1', 'Param 2'], "Test export to csv: Exported fields are wrong"

        for row in csv_reader:
            assert row['Param 1'] == '[20.0, 20.0]', "Test export to csv: Float export gone wrong"
            assert row['Param 2'] == "['abcd', 'abcd']", "Test export to csv: String export gone wrong"