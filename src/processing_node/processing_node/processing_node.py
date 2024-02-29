import rclpy
from rcl_interfaces.msg import SetParametersResult

from processing_node.ml_deploy_library import *
from processing_node.config_handler_node import ConfigHandlerNode


class ProcessingNode(ConfigHandlerNode):
    """
    ROS2 node that sets up input subscribers and output publishers according
    to a config and handles the data transfer to a processing function

    Attributes:
        publisher_dict (dict): dict containing the mapping information of output data to 
        respective output publisher

        processor (object): Object that contains the function to be executed 
    """

    def __init__(self, processor: object, config_path:str=None, frequency: float=30.0, node_handle:str="processing_node"):
        """
        Initializes the ProcessingNode with a given function.

        Args:
            processor (object): Object that contains the function to be executed.
            config_path (str): Path to the config file.
            frequency (float): Frequency at which the function should be called.
        """

        # init base class
        super().__init__(config_path, frequency, node_handle)

        check_processor(processor)

        self.publisher_dict = self.set_up_publishers(self._output_topic_dict)

        self.processor = processor
        self._declare_parameters_for_processor(processor)

    def _declare_parameters_for_processor(self, processor):
        """
        Declares ROS2 parameters for this node based on a given function's type annotations.

        Args:
            processor (object): Object that contains the function to be executed.
        """

        parameters = processor.get_parameters()
        rclpy.logging.get_logger("processing_node").info(f"Parameters: {parameters}")

        if parameters is None:
            return

        for param_name, param_type  in parameters.items():
            if self.has_parameter(param_name):
                self.get_logger().warn(f"Parameter {param_name} is already declared.")
                continue

            default_values = {int: 0,
                              float: 0.0,
                              str: "",
                              bool: False}

            if param_type not in default_values:
                self.get_logger().error(f"Parameter type {param_type} is not supported.")
                continue

            self.declare_parameter(param_name, default_values[param_type])

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self,params):
        """
        Callback function that is called whenever a parameter is set
        
        Args:
            params (list): list of parameters that were set
            
        Returns:
            SetParametersResult: ROS2 result object that indicates 
                                 whether the parameter setting was successful
            
        """
        param_dict = {}
        for individual_param in params:
            param_dict[individual_param.name] = individual_param.value

        self.processor.set_parameters(param_dict)
        return SetParametersResult(successful=True)

    def set_up_publishers(self, topic_dict: dict) -> dict:
        """
        Sets up publishers according to the outputs specified in the config

        Args:
            topic_dict (dict): dict that contains information about the topics to be published
        
        Returns:
            output_dict (dict): dict specifying the publisher for each publishable output and
            in what field of the message to publish which output
        """

        topics_to_publish_dict = {}

        for topic_name in topic_dict:
            message_type_string = topic_dict[topic_name]['MessageType']
            if message_type_string not in self.supported_message_types:
                self.get_logger().warn(f'Topic name {topic_name} cannot be published, Message Type is unknown')
            else:
                topics_to_publish_dict[topic_name] = {'Publisher': self.create_publisher(self.supported_message_types[message_type_string],
                                                                                        topic_name,
                                                                                        10)}

                topics_to_publish_dict[topic_name]['MessageType'] = message_type_string
                topics_to_publish_dict[topic_name]['Fields'] = {}

                for key in topic_dict[topic_name]:
                    if key != 'MessageType':
                        topics_to_publish_dict[topic_name]['Fields'][key] = topic_dict[topic_name][key]

        return topics_to_publish_dict

    def execute(self):
        """
        Function that is called on a timer, sends collected input data to the 
        processing function and publishes the resulting output
        """
        max_length = 0

        if self.aggregated_input_data == dict.fromkeys(self.aggregated_input_data.keys(), []):
            return

        #  Call processing function
        processed_data = self.processor.execute(self.aggregated_input_data)

        # Reset data dict
        self.aggregated_input_data = dict.fromkeys(self.aggregated_input_data.keys(), [])

        # Publish output
        if processed_data is not None:
            for topic in self.publisher_dict:
                message_type_string = self.publisher_dict[topic]['MessageType']
                message_type = self.supported_message_types[message_type_string]
                output_msg = message_type()
                for output in self.publisher_dict[topic]['Fields']:
                    field = self.publisher_dict[topic]['Fields'][output]
                    # Check if no further field is specified, in which case the output data 
                    # is expected to be a publishable message type
                    if field == 'FullMessage':
                        output_msg = processed_data[output]
                    else:
                        # Check if field is only a string and the message therefore only 1 level deep
                        if isinstance(field, str):
                            field = [field]
                        try:
                            data = processed_data[output]
                            output_msg = set_nested_attribute(output_msg, field, data)
                        except:
                            self.get_logger().warn(f'Output {output} not found in model output or wrong data type')
                try:
                    self.publisher_dict[topic]['Publisher'].publish(output_msg)
                except:
                    self.get_logger().warn(f'Problem publishing topic {topic}, wrong data type?')
