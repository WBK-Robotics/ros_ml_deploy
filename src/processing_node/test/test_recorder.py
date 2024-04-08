import rclpy
import unittest
import csv
import shutil
import os

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

        self.out_folder = f"{os.getcwd()}/ros_ml_test_csvs"
        try:
            os.mkdir(self.out_folder)
        except:
            pass

        second_executor = SingleThreadedExecutor()
        recorder_node = RecorderNode(get_config_file_path('recorder_test_config.yaml'), self.out_folder, number_of_input_points=500)

        second_executor.add_node(self.node)
        second_executor.add_node(recorder_node)

        msg_float = Float32()
        msg_float.data = 20.0

        msg_string = String()
        msg_string.data = "abcd"

        for i in range(500):
            pub_float.publish(msg_float)
            second_executor.spin_once()
            if i%2==0:
                pub_string.publish(msg_string)
                second_executor.spin_once()

            if recorder_node.recording_done.done():
                break

        second_executor.spin_once()
        second_executor.spin_once()
        second_executor.spin_once()
        
        recorder_node.write_to_files()
        recorder_node.destroy_node()

    def tearDown(self):
        self.node.destroy_node()
        shutil.rmtree(self.out_folder)

    def test_file_output(self):
        with open(f'{self.out_folder}/record_data_float.csv', 'r') as file:
            csv_reader = csv.DictReader(file)

            assert csv_reader.fieldnames == ['Param 1'], "Test export to csv (Float): fields are wrong"
            row_counter = 0
            for row in csv_reader:
                row_counter += 1
                
            assert row_counter == 500, "Test export to csv: Float export gone wrong"

        with open(f'{self.out_folder}/record_data_string.csv', 'r') as file:
            csv_reader = csv.DictReader(file)

            assert csv_reader.fieldnames == ['Param 2'], "Test export to csv (String): fields are wrong"

            row_counter = 0
            for row in csv_reader:
                row_counter += 1
                
            assert row_counter == 250, "Test export to csv: String export gone wrong"