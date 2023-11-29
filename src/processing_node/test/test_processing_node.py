import pytest
import yaml
from processing_node import ProcessingNode

import os
import ament_index_python.packages


def get_config_file_path(filename):
    # Get the directory of the package
    package_directory = ament_index_python.packages.get_package_share_directory('processing_node')

    # Construct the full path to the configuration file
    config_file_path = os.path.join(package_directory, 'config', filename)

    return config_file_path


def test_valid_configuration_file():
    tmp_path = get_config_file_path("config.yaml")
    # Arrange
    config_dict = {
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
    config_path = tmp_path / "config.yaml"
    with open(config_path, "w") as file:
        yaml.dump(config_dict, file)
    
    # Act
    node = ProcessingNode(lambda x: x)  # Dummy function
    loaded_config = node.load_config(str(config_path))
    
    # Assert
    assert loaded_config == config_dict
