import rclpy
import csv

from asyncio import Future
from rclpy.node import Node

from processing_node.processing_node import ProcessingNode
from processing_node.ml_deploy_library import *

recording_done = Future()

class Recorder:

    def __init__(self):
        self.parameters = {"number_of_input_points" : int}
    
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
        print(number_of_input_points)
        if number_of_input_points >= self.parameters["number_of_input_points"]:
            with open('/root/mounted_folder/Files/testfile.csv', 'w') as f:
                w = csv.writer(f)
                w.writerows(aggregated_data_dict.items())
            recording_done.set_result("Done")
        

def main():
    rclpy.init(args=None)
    recorder_object = Recorder()
    config_path = '/root/mounted_folder/ros_ml_deploy/config/config.yaml'
    config = load_config(config_path)
    if 'Outputs' in config:
        config.pop('Outputs')
    
    recorder_node = ProcessingNode(recorder_object, config=config,frequency=200)
    recorder_object.set_parameters({"number_of_input_points":5})

    rclpy.spin_until_future_complete(recorder_node, recording_done)