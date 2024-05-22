import csv
import os
import argparse

from asyncio import Future

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
        recording_done (Future): Future object that is set to done when the number of
        input points set during construction or by altering of the respective ROS-parameter
        is reached and allows termination of node execution
    """

    def __init__(self, config_path:str=None, path_to_csv_folder:str=None, number_of_input_points:int=1000, frequency: float=30.0):
        """
        Initializes the recorder node.

        Args:
            config_path (str): Path to the config specifiying the inputs and imports
            outfile (StringIO): File-like object to write the data into
            number_of_input_poins (int): Number of points to be written into the outfile.
            frequency (float): Dictates how often the execute method is called
        """

        super().__init__(config_path, frequency, "recorder_node")

        self.output_folder_path = path_to_csv_folder

        self.recording_done = Future()

        self.declare_parameter("number_of_input_points", number_of_input_points)

    def execute(self):
        """
        Function that is called on a timer, checks if the minimum number of input points has
        been recorded, writes the data into the outfile and sets recording_done to "Done"
        """
        actual_number_of_input_points = len(max(self.aggregated_input_data.values(), key=len))
        should_number_of_input_points = self.get_parameter("number_of_input_points").value

        if actual_number_of_input_points >= should_number_of_input_points >= 0:
            for key in self.aggregated_input_data.keys():
                self.aggregated_input_data[key] = self.aggregated_input_data[key][:should_number_of_input_points]
            self.write_to_files()
            self.recording_done.set_result("Done")

    def write_to_files(self):
        """
        Function that writes output files
        """
        actual_number_of_input_points = len(max(self.aggregated_input_data.values(), key=len))
        should_number_of_input_points = self.get_parameter("number_of_input_points").value

        if actual_number_of_input_points >= should_number_of_input_points >= 0:
            for key in self.aggregated_input_data.keys():
                self.aggregated_input_data[key] = self.aggregated_input_data[key][:should_number_of_input_points]

        for topic in self._input_topic_dict:
            with open(f"{self.output_folder_path}/{topic}.csv", "w") as file:
                writer = csv.writer(file)
                header = list(self._input_topic_dict[topic].keys())
                header.remove('MessageType')

                writer.writerow(header)
                for element_number in range(len(self.aggregated_input_data[header[0]])):
                    row = []
                    for key in header:
                        row.append(self.aggregated_input_data[key][element_number])
                    writer.writerow(row)

def main():
    """
    Method used as an entrypoint.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--out_folder", help="Needed path to output folder")
    parser.add_argument("--config_path", help="Optional config path, if none is specified defaults to config/config.yaml")
    parser.add_argument("--num", help="Optional Number of input points, if none specified defaults to -1 (recording until manual stoppage)")

    args=parser.parse_args()

    try:
        path_to_csv_folder = args.out_folder
    except:
        print("Missing argument: path to csv folder")
        return()

    if not os.path.isdir(path_to_csv_folder):
        os.mkdir(path_to_csv_folder)

    if args.num is not None:
        number_of_input_points = args.num
    else:
        number_of_input_points = -1

    if args.config_path is not None:
        config_path = args.config_path
    else:
        config_path = get_config_file_path('config.yaml')

    rclpy.init(args=None)

    recorder_node = RecorderNode(config_path, path_to_csv_folder, number_of_input_points, frequency=200)

    try:
        rclpy.spin_until_future_complete(recorder_node, recorder_node.recording_done)
    except KeyboardInterrupt:
        recorder_node.write_to_files()
