import sys
import importlib
import time
import inspect
import os

import rclpy
import yaml
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ros2topic.api import get_msg_class


def check_if_config_is_valid(config: dict):
    """
    Method that checks if the given config has:
    - Inputs without a 'Topic' or 'Field'
    - Outputs without a 'Topic' or 'Field' or 'MessageType'
    - 'MessageType' in 'Outputs' that are not featured in 'Imports'
    - 'Imports' without a 'Package' or 'Module'

    Args:
        config (dict): The config dict to be checked
    """
    config_is_valid = True

    error_message = ""

    if "Inputs" not in config:
        config["Inputs"] = []

    for input_name in config["Inputs"]:
        for necessary_part in ["Topic", "Field"]:
            if necessary_part not in config["Inputs"][input_name]:
                error_message += f"Config Format Error: Input {input_name} has no '{necessary_part}' \n"
                config_is_valid = False

    if "Imports" not in config:
        config["Imports"] = []

    for import_name in config["Imports"]:
        for necessary_part in ["Package", "Module"]:
            if necessary_part not in config["Imports"][import_name]:
                error_message += f"Config Format Error: Import {import_name} has no '{necessary_part}' \n"
                config_is_valid = False

    if "Outputs" not in config:
        config["Outputs"] = []

    for output_name in config["Outputs"]:
        for necessary_part in ["Topic", "Field"]:
            if necessary_part not in config["Outputs"][output_name]:
                error_message += f"Config Format Error: Output {output_name} has no '{necessary_part}' \n"
                config_is_valid = False
        if "MessageType" not in config["Outputs"][output_name]:
            error_message += f"Config Format Error: Output {output_name} has no 'MessageType' \n"
            config_is_valid = False
        elif config["Outputs"][output_name]["MessageType"] not in config["Imports"]:
            error_message += f"Config Format Error: Message Type {config['Outputs'][output_name]['MessageType']} requested by Output '{output_name}' not in 'Imports' \n"
            config_is_valid = False

    return config_is_valid, error_message

def map_input_and_output_names_to_topics(config: dict) -> tuple[dict, dict]:
        """
        Load the actual mappings of input and output names

        Args:
            config (dict): Config dict that specifies requested inputs and outputs and relevant 
            information about those inputs and outputs
        
        Returns:
            input topic (dict): Dict specifying which topics to subscribe to and what fields of 
            those topics are carrying which input information

            output topic (dict): Dict specifying which topics to publish and what fields of those 
            topics carry what information
        """

        input_topic_dict = {}
        output_topic_dict = {}

        for key in config['Inputs']:
            topic = config['Inputs'][key]['Topic']
            field = config['Inputs'][key]['Field']
            if topic not in input_topic_dict:
                input_topic_dict[topic] = {key: field}
            else:
                input_topic_dict[topic][key] = field

        for key in config['Outputs']:
            topic = config['Outputs'][key]['Topic']
            field = config['Outputs'][key]['Field']
            if topic not in output_topic_dict:
                output_topic_dict[topic] = {key: field}
                output_topic_dict[topic]['MessageType'] = config['Outputs'][key]['MessageType']
            else:
                output_topic_dict[topic][key] = field

        return input_topic_dict, output_topic_dict

def check_processor(processor: object) -> bool:
    """
    Check if the given object is a valid processor object

    Args:
        processor (object): Object to be checked

    Returns:
        bool: True if the given object is a valid processor object, False otherwise

    Raises:
        ValueError: If the given object is not a valid processor object
    """
    has_execute = callable(getattr(processor, 'execute', None))
    if not has_execute:
        raise ValueError("The processor object must be a class which has a callable execute function.")
    has_set_parameters = callable(getattr(processor, 'set_parameters', None))
    if not has_set_parameters:
        raise ValueError("The processor object must be a class which has a callable set_parameters function.")
    has_get_parameters = callable(getattr(processor, 'get_parameters', None))
    if not has_get_parameters:
        raise ValueError("The processor object must be a class which has a callable get_parameters function.")
    
    return True

def load_config(config_path: str) -> dict:
    """
    Loads config from path and returns it as a dict

    Args:
        config_path (String): Absolute path to the config file expected to be .yaml
    
    Returns:
        config dict (dict): Contents of the config.yaml restructured into a dict
    """
    if not isinstance(config_path,str):
            raise ValueError("The config_path must be a string.")
    if not os.path.isfile(config_path):
        raise ValueError("The config_path must be a valid file path.")
    # Get dict of inputs from parameters
    with open(config_path, 'r') as file:
        try:
            config = yaml.safe_load(file)
        except yaml.YAMLError as e:
            raise ValueError("While the config_path exists, it does not lead to a valid yaml file.")

    return config

def import_needed_modules( config: dict):
    """
    Imports Modules specified in config dict (needed for output message types)
    Also adds the imported modules to the supported messsage types for output dict

    Args:
        config (dict): Config dict that specifies which modules to import from where
    """
    supported_message_types_to_publish = {}
    try:
        import_dict = config['Imports']
        for to_import in import_dict:
            package = import_dict[to_import]["Package"]
            module = import_dict[to_import]["Module"]
            message_type_class = getattr(importlib.import_module(package), module)
            supported_message_types_to_publish[to_import] = message_type_class
        return supported_message_types_to_publish
    except:
        raise ValueError("Import of message based modules specified in config failed!")


class ProcessingNode(Node):
    """
    Ros2 node that sets up input subscribers and output publishers according
    to a config and handles the data transfer to a processing function

    Attributes:
        supported_message_types_to_publish (dict): dict that is filled with imported message types
        
        aggregated_input_data (dict): dict that is filled with data gathered from
        the set up subscribers to be used as input data for the processing model

        publisher_dict (dict): dict containing the mapping information of output data to 
        respective output publisher

        function_to_execute (function): the processing function to be executed 

        timer (Timer): Timer that calls function_to_execute with a frequency specified
        in construction   
    """

    def __init__(self, processor: callable, config_path:str=None, frequency: float=30.0):
        """
        Initializes the ProcessingNode with a given function.

        Args:
            func (function): Function with type annotations to set as parameters.
            frequency (float): Frequency at which the function should be called.
        """

        # init both base classes
        Node.__init__(self, "processing_node")
        check_processor(processor)

        self._config = load_config(config_path)
        # check that the config file is valid:
        config_is_valid, error_message = check_if_config_is_valid(self._config)
        if not config_is_valid:
            raise ValueError(error_message)

        self.supported_message_types_to_publish= import_needed_modules(self._config)

        
        self.aggregated_input_data = dict.fromkeys(self._config['Inputs'].keys(), [])

        
        input_topic_dict, output_topic_dict = map_input_and_output_names_to_topics(self._config)

        self.set_up_subscriptions(input_topic_dict)
        self.publisher_dict = self.set_up_publishers(output_topic_dict)

        self.processor = processor
        self._declare_parameters_for_function(processor)

        # Set model timer period
        timer_period = 1.0 / frequency

        # Create timer that calls the processing function
        self.timer = self.create_timer(timer_period, self.execute_function)

    def listener_callback(self, msg, field_names: dict):
        """
        Function that is called whenever something is published to a subscribed topic 
        and modifies the data dict accordingly

        Args:
            msg (ROS2 Message or other struct): Message that triggered the function call
            field_names (dict): Dict that specifies which fields are carrying what input information
        """

        # Read the relevant message field and append it to the relevant list in the data dict
        for input_name in field_names:
            # Loop over attributes to reach deeper message levels until the actual data is reached
            base = msg
            # Check if field name is a string and the message therefore only 1 level deep
            if isinstance(field_names[input_name], str):
                base = getattr(base, field_names[input_name])
            else:
                for i in range(len(field_names[input_name])):
                    attribute = field_names[input_name][i]
                    base = getattr(base, attribute)
            # Does not work with append for whatever reason
            self.aggregated_input_data[input_name] = self.aggregated_input_data[input_name] + [base]

    def _declare_parameters_for_function(self, processor):
        """
        Declares ROS2 parameters for this node based on a given function's type annotations.

        Args:
            func (function): Function with type annotations to set as parameters.
        """

        parameters = processor.get_parameters()
        rclpy.logging.get_logger("processing_node").info(f"Parameters: {parameters}")
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
        param_dict = {}
        for individual_param in params:
            param_dict[individual_param.name] = individual_param.value

        self.processor.set_parameters(param_dict)        
        return SetParametersResult(successful=True)

   

    

    def set_up_subscriptions(self, topic_dict: dict):
        """
        Sets up Subscriptions to topics that are mentioned in topic_dict, 
        also sets up listener_callback with correct input-message field mapping

        Args:
            topic_dict (dict): dict that contains the topics to be subscribed to,
            the inputs they carry and in which field these inputs are
        """

        for topic_name in topic_dict:
            subscribed = False
            timeout = time.time() + 4
            while not subscribed:
                # Get all currently publishing topics
                topic_list = self.get_topic_names_and_types()
                # Check if topic we want is in currently publishing topics
                for topic_tuple in topic_list:
                    if topic_name in topic_tuple[0]:
                        # Get msg type
                        msg_type = get_msg_class(self, topic_tuple[0], include_hidden_topics=True)
                        # Set up subscription
                        self.create_subscription(msg_type,
                                                topic_tuple[0],
                                                lambda msg, field_names=topic_dict[topic_name] : self.listener_callback(msg, field_names),
                                                10)
                        # Toggle bool
                        subscribed = True
                        self.get_logger().info(f"Subscribed to topic {topic_name} which publishes {list(topic_dict[topic_name])}")
                if time.time() > timeout:
                    self.get_logger().warning(f"Input topic {topic_name} was not found within 60 Seconds, no subscription was set up")
                    break

                # Wait half a second
                time.sleep(0.5)

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
            if message_type_string not in self.supported_message_types_to_publish:
                self.get_logger().warn(f'Topic name {topic_name} cannot be published, Message Type is unknown')
            else:
                topics_to_publish_dict[topic_name] = {'Publisher': self.create_publisher(self.supported_message_types_to_publish[message_type_string],
                                                                                        topic_name,
                                                                                        10)}

                topics_to_publish_dict[topic_name]['MessageType'] = message_type_string
                topics_to_publish_dict[topic_name]['Fields'] = {}

                for key in topic_dict[topic_name]:
                    if key != 'MessageType':
                        topics_to_publish_dict[topic_name]['Fields'][key] = topic_dict[topic_name][key]

        return topics_to_publish_dict

    @staticmethod
    def set_nested_attribute(parent: object, part_list: list, new_value:object):
        """
        Helper function that sets an attribute within a nested object structure

        Args:
            parent (object): object for which an attribute is to be set
            
            part_list (list): path to the requested attribute in the form of ["nested_1", 
            "nested_2", "attribute"] without restrictions on length

            new_value (Attribute data type): the value the attribute is to be set to
        
        Returns:
            parent (object): object with the now set attribute
        """
        final_attribute_index = len(part_list)-1
        current_attribute = parent
        for i, part in enumerate(part_list):
            new_attribute = getattr(current_attribute, part)
            if i == final_attribute_index:
                setattr(current_attribute, part, new_value)
            current_attribute = new_attribute

        return parent    

    def execute_function(self):
        """
        Function that is called on a timer, sends collected input data to the 
        processing function and publishes the resulting output
        """

        #  Call processing function
        processed_data = self.processor.execute(self.aggregated_input_data)

        # Reset data dict
        self.aggregated_input_data = dict.fromkeys(self.aggregated_input_data.keys(), [])

        # Publish output
        if processed_data is not None:
            for topic in self.publisher_dict:
                message_type_string = self.publisher_dict[topic]['MessageType']
                message_type = self.supported_message_types_to_publish[message_type_string]
                output_msg = message_type()
                for output in self.publisher_dict[topic]['Fields']:
                    field = self.publisher_dict[topic]['Fields'][output]
                    # Check if field is only a string and the message therefore only 1 level deep
                    if isinstance(field, str):
                        field = [field]
                    try:
                        data = processed_data[output]
                        output_msg = self.set_nested_attribute(output_msg, field, data)
                        self.publisher_dict[topic]['Publisher'].publish(output_msg)
                    except:
                        self.get_logger().warn(f'Output {output} not found in model output or wrong data type')

