import rclpy
import csv
import sys

from rclpy.node import Node

from processing_node.processing_node import ProcessingNode
from processing_node.ml_deploy_library import *

class Recorder:

    def __init__(self):
        pass
    
    def set_parameters(self):
        # TODO: Possibly set wanted amount of data here?
        pass
    
    def get_parameters(self):
        pass

    def execute(self, aggregated_data_dict):
        with open('/root/mounted_folder/Files/testfile.csv', 'w') as f:
            w = csv.writer(f)
            w.writerows(aggregated_data_dict.items())

def main():
    rclpy.init(args=None)
    recorder_object = Recorder()
    config_path = '/root/mounted_folder/ros_ml_deploy/config/config.yaml'
    config = load_config(config_path)
    if 'Outputs' in config:
        config.pop('Outputs')
    
    processsing_node = ProcessingNode(recorder_object, config=config,frequency=1)

    rclpy.spin(processsing_node)

# TODO: Manual config loading, removing outputs from config, setting up function with csv export