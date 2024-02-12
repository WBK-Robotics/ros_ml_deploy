import importlib
import os

import yaml

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
        for necessary_part in ["Topic"]:
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
        for necessary_part in ["Topic"]:
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
            if 'Field' in config['Inputs'][key]:
                field = config['Inputs'][key]['Field']
                if topic not in input_topic_dict:
                    input_topic_dict[topic] = {key: field}
                    input_topic_dict[topic]['MessageType'] = config['Inputs'][key]['MessageType']
                else:
                    input_topic_dict[topic][key] = field
            else:
                input_topic_dict[topic] = {key: 'FullMessage'}
                input_topic_dict[topic]['MessageType'] = config['Inputs'][key]['MessageType']

        for key in config['Outputs']:
            topic = config['Outputs'][key]['Topic']
            if 'Field' in config['Outputs'][key]:
                field = config['Outputs'][key]['Field']
                if topic not in output_topic_dict:
                    output_topic_dict[topic] = {key: field}
                    output_topic_dict[topic]['MessageType'] = config['Outputs'][key]['MessageType']
                else:
                    output_topic_dict[topic][key] = field
            else:
                output_topic_dict[topic] = {key: 'FullMessage'}
                output_topic_dict[topic]['MessageType'] = config['Ouptuts'][key]['MessageType']

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

