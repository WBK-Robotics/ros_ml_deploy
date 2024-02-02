import rclpy

from rclpy.node import Node

from processing_node.ml_deploy_library import *

class RecorderNode(Node):
    """
    ROS 2 Node that sets up subscribers according to a config and gathers incoming data
    into a file
    
    Attributes: 
        aggregated_input_data (dict):  dict that is filled with data gathered from
        the set up subscribers to be saved into a file
    """

    def __init__(self, config_path:str=None):
        """
        Initializes the Recorder Node.
        
        Args:
            config_path (str): Path to the config file.
        """
        Node.__init__(self, "recorder_node")

        self._config = load_config(config_path)
        # check that the config file is valid:
        config_is_valid, error_message = check_if_config_is_valid(self._config)
        if not config_is_valid:
            raise ValueError(error_message)
    