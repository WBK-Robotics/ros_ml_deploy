import csv
import sys
import os

from asyncio import Future
from io import StringIO

import ament_index_python
import rclpy

from processing_node.config_handler_node import ConfigHandlerNode

def get_config_file_path(filename):
    # Get the directory of the package
    package_directory = ament_index_python.packages.get_package_share_directory('processing_node')

    # Construct the full path to the configuration file
    config_file_path = os.path.join(package_directory, 'config', filename)

    return config_file_path

class RecorderNode(ConfigHandlerNode):
    """
    ROS2 node that sets up input subscribers according to a config and writes
    the there published data into an outfile

    Attributes:
        outfile (StringIO): File-like Object to write the data into

        recording_done (Future): Future object that is set to done when the number of
        input points set during construction or by altering of the respective ROS-parameter
        is reached and allows termination of node execution
    """

    def __init__(self, config_path:str=None, outfile:StringIO=None, number_of_input_points:int=1000, frequency: float=30.0):
        """
        Initializes the recorder node.

        Args:
            config_path (str): Path to the config specifiying the inputs and imports
            outfile (StringIO): File-like object to write the data into
            number_of_input_poins (int): Number of points to be written into the outfile.
            frequency (float): Dictates how often the execute method is called
        """

        super().__init__(config_path, frequency, "recorder_node")

        self.outfile = outfile

        self.recording_done = Future()

        self.declare_parameter("number_of_input_points", number_of_input_points)

    def execute(self):
        """
        Function that is called on a timer, checks if the minimum number of input points has
        been achived, writes the data into the outfile and sets recording_done to "Done"
        """
        actual_number_of_input_points = len(max(self.aggregated_input_data.values(), key=len))
        should_number_of_input_points = self.get_parameter("number_of_input_points").value

        if actual_number_of_input_points >= should_number_of_input_points:
            for key in self.aggregated_input_data.keys():
                self.aggregated_input_data[key] = self.aggregated_input_data[key][:should_number_of_input_points]
            w = csv.DictWriter(self.outfile, self.aggregated_input_data.keys())
            w.writeheader()
            w.writerow(self.aggregated_input_data)
            self.recording_done.set_result("Done")


def main():
    """
    Method used as an entrypoint.
    """

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

    return()
