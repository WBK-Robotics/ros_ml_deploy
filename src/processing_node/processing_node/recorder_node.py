import rclpy
import csv
import sys
import ament_index_python

from io import StringIO
from asyncio import Future

from processing_node.processing_node import ProcessingNode
from processing_node.ml_deploy_library import *

def get_config_file_path(filename):
    # Get the directory of the package
    package_directory = ament_index_python.packages.get_package_share_directory('processing_node')

    # Construct the full path to the configuration file
    config_file_path = os.path.join(package_directory, 'config', filename)

    return config_file_path

class RecorderNode(ProcessingNode):

    def __init__(self, config_path:str=None, outfile:StringIO=None, frequency: float=30.0):
        rclpy.node.Node.__init__(self, "recorder_node")

        self._config = load_config(config_path)

        config_is_valid, error_message = check_if_config_is_valid(self._config)
        if not config_is_valid:
            raise ValueError(error_message)
        
        self.outfile = outfile

        self.supported_message_types = import_needed_modules(self._config)
        self.aggregated_input_data = dict.fromkeys(self._config['Inputs'].keys(), [])

        input_topic_dict = map_input_and_output_names_to_topics(self._config)[0]
        self.set_up_subscriptions(input_topic_dict)

        self.declare_parameter("number_of_input_points", 1000)

        # Set model timer period
        timer_period = 1.0 / frequency

        self.timer = self.create_timer(timer_period, self.execute)

        self.recording_done = Future()

    def execute(self):
        number_of_input_points = len(max(self.aggregated_input_data.values(), key=len))
        if number_of_input_points >= self.get_parameter("number_of_input_points").value:
            w = csv.writer(self.outfile)
            w.writerows(self.aggregated_input_data.items())
            self.recording_done.set_result("Done")
        

def main():

    try:
        path_to_csv = sys.argv[1]
    except IndexError:
        print("Missing argument: path to csv")
        return()
    
    rclpy.init(args=None)

    config_path = get_config_file_path('config.yaml')

    with open(path_to_csv, 'w') as csv_out_file:
        recorder_node = RecorderNode(config_path, csv_out_file, frequency=200)

        rclpy.spin_until_future_complete(recorder_node, recorder_node.recording_done)