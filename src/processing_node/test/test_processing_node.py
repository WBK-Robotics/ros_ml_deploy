from typing import Any
import pytest
import yaml
from processing_node.processing_node import ProcessingNode, check_if_config_is_valid, map_input_and_output_names_to_topics
import rclpy
import os
import ament_index_python.packages

from unittest.mock import MagicMock


class SomeProcessor:
    def __init__(self):
        self.parameters= {"float parameter": float, "int parameter": int}
    def set_parameters(self, parameters):
        self.parameters = parameters
    
    def get_parameters(self):
        return self.parameters
    
    def execute(self, main_value, parameters):
        return main_value, parameters


valid_config = {
    "Inputs": {
        "Motor Current 1": {
            "Description": "Example input parameter, for instance emulating a motor current",
            "Topic": "sensor_data",
            "Field": ["data"]
        }
    },
    "Outputs": {
        "float parameter": {
            "Description": "Example float parameter",
            "Topic": "ExampleFloat",
            "Field": ["data"],
            "MessageType": "Float32"
        },
        "int parameter": {
            "Description": "Example int parameter",
            "Topic": "ExampleInt",
            "Field": ["data"],
            "MessageType": "Int16"
        }
    },
    "Imports": {
        "Float32": {
            "Package": "std_msgs.msg",
            "Module": "Float32"
        },
        "Int16": {
            "Package": "std_msgs.msg",
            "Module": "Int16"
        }
    }
}


invalid_config = {
    "Inputs": {
        "Motor Current 1": {
            "Description": "Missing Topic and Field",
        }
    },
    "Outputs": {
        "float parameter": {
            "Description": "Missing Topic, Field, and MessageType",
        }
    },

}

def get_config_file_path(filename):
    # Get the directory of the package
    package_directory = ament_index_python.packages.get_package_share_directory('processing_node')

    # Construct the full path to the configuration file
    config_file_path = os.path.join(package_directory, 'test','test_config', filename)

    return config_file_path

config_path = get_config_file_path("config.yaml")




def test_config_validation():

    assert check_if_config_is_valid(valid_config)[0] == True
    assert check_if_config_is_valid(invalid_config)[0] == False


def test_map_input_and_output_names_to_topics():
    expected_input_topic_dict = {
        "sensor_data": {
            "Motor Current 1": ["data"]
        }
    }
    expected_output_topic_dict = {
        "ExampleFloat": {
            "float parameter": ["data"],
            "MessageType": "Float32"
        },
        "ExampleInt": {
            "int parameter": ["data"],
            "MessageType": "Int16"
        }
    }
    received_input_topic_dict, received_output_topic_dict = map_input_and_output_names_to_topics(valid_config)

    assert received_input_topic_dict == expected_input_topic_dict
    assert received_output_topic_dict == expected_output_topic_dict

def test_call_function_with_current_parameters():

    
    rclpy.init()
    some_processor = SomeProcessor()
    node = ProcessingNode(some_processor ,config_path=config_path)
    node.get_parameter = MagicMock(side_effect=lambda param: MagicMock(value=param))
    test_input = 42 # Example input

    result = node.call_function_with_current_parameters(test_input)

    expected = test_input, {"float parameter": "float parameter", "int parameter": "int parameter"}
    assert result == expected
    rclpy.shutdown()


def test_config_loading():
    rclpy.init()

    with open(config_path, "w") as file:
        yaml.dump(valid_config, file)
    
    some_processor = SomeProcessor()
    node = ProcessingNode(some_processor ,config_path=config_path)  # Dummy function
    loaded_config = node.load_config(str(config_path))
    
    assert loaded_config == valid_config
    rclpy.shutdown()




if __name__ == '__main__':
    pytest.main(['-vv', '-s', __file__])