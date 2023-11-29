import pytest
import yaml
from processing_node.processing_node import ProcessingNode, check_if_config_is_valid
import rclpy
import os
import ament_index_python.packages



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

def test_config_validation():
    # Arrange
    # Act
    # Assert
    assert check_if_config_is_valid(valid_config)[0] == True
    assert check_if_config_is_valid(invalid_config)[0] == False



def test_config_loading():
    rclpy.init()
    config_path = get_config_file_path("config.yaml")
    # Arrange
    

    with open(config_path, "w") as file:
        yaml.dump(valid_config, file)
    
    # Act
    node = ProcessingNode(lambda x: x,config_path=config_path)  # Dummy function
    loaded_config = node.load_config(str(config_path))
    
    # Assert
    assert loaded_config == valid_config


if __name__ == '__main__':
    pytest.main(['-vv', '-s', __file__])