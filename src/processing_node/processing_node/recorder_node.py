import rclpy
import csv
import sys
import ament_index_python

from asyncio import Future

from processing_node.processing_node import ProcessingNode
from processing_node.ml_deploy_library import *

recording_done = Future()

def get_config_file_path(filename):
    # Get the directory of the package
    package_directory = ament_index_python.packages.get_package_share_directory('processing_node')

    # Construct the full path to the configuration file
    config_file_path = os.path.join(package_directory, 'config', filename)

    return config_file_path

class Recorder:

    def __init__(self):
        self.parameters = {"number_of_input_points" : int, "path_to_csv": str}
    
    def set_parameters(self, parameters:dict):
        for key in parameters.keys():
            if key in self.parameters.keys():
                self.parameters[key] = parameters[key]
            else:
                raise ValueError("The parameter {} is not supported by this processor".format(key))
    
    def get_parameters(self):
        return self.parameters

    def execute(self, aggregated_data_dict):
        number_of_input_points = len(max(aggregated_data_dict.values(), key=len))
        if number_of_input_points >= self.parameters["number_of_input_points"]:
            with open(self.parameters['path_to_csv'], 'w') as f:
                w = csv.writer(f)
                w.writerows(aggregated_data_dict.items())
            recording_done.set_result("Done")
        

def main():

    try:
        path_to_csv = sys.argv[1]
    except IndexError:
        print("Missing argument: path to csv")
        return()
    
    rclpy.init(args=None)
    recorder_object = Recorder()

    config_path = get_config_file_path('config.yaml')

    config = load_config(config_path)
    check_if_config_is_valid(config)
    if 'Outputs' in config:
        config.pop('Outputs')
    
    recorder_node = ProcessingNode(recorder_object, config=config,frequency=200)
    recorder_object.set_parameters({"number_of_input_points":5, "path_to_csv":path_to_csv})

    rclpy.spin_until_future_complete(recorder_node, recording_done)