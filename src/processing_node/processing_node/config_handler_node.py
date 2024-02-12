from rclpy.node import Node

from processing_node.ml_deploy_library import *

class ConfigHandlerNode(Node):
    """
    ROS2 node that handles a config containing input and output variables and their respective
    topics. It serves as a base class for both the RecorderNode and the ProcessingNode.

    Attributes:
        supported_message_types (dict): dict that is filled with imported message types
        
        aggregated_input_data (dict): dict that is filled with data gathered from
        the set up subscribers to be used as input data for the processing model

        _input_topic_dict (dict): dict that maps input topics to variable names

        _output_topic_dict (dict): dict that maps output topics to variable names

        timer (Timer): Timer that calls function_to_execute with a frequency specified
        in construction
    """

    def __init__(self, config_path:str=None, frequency:float=30.0, node_handle:str="config_handler_node"):
        """
        Initializes the config handler node.

        Args:
            config_path (str): Path to the config specifiying the inputs and imports
            frequency (float): Dictates how often the execute method is called
            node_handle (str): Name of the node to be constructed
        """

        super().__init__(node_handle)

        self._config = load_config(config_path)

        # check that the config file is valid:
        config_is_valid, error_message = check_if_config_is_valid(self._config)
        if not config_is_valid:
            raise ValueError(error_message)

        self.supported_message_types = import_needed_modules(self._config)

        self.aggregated_input_data = dict.fromkeys(self._config['Inputs'].keys(), [])

        self._input_topic_dict, self._output_topic_dict = map_input_and_output_names_to_topics(self._config)

        self.set_up_subscriptions(self._input_topic_dict)

        timer_period = 1.0 / frequency

        # Create timer that calls the processing function
        self.timer = self.create_timer(timer_period, self.execute)

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
            if input_name != 'MessageType':
                # Loop over attributes to reach deeper message levels until the actual data is reached
                base = msg
                # Check if field name is a string and the message therefore only 1 level deep
                if isinstance(field_names[input_name], str):
                    if field_names[input_name] != 'FullMessage':
                        base = getattr(base, field_names[input_name])
                else:
                    for i in range(len(field_names[input_name])):
                        attribute = field_names[input_name][i]
                        base = getattr(base, attribute)
                # Does not work with append for whatever reason
                self.aggregated_input_data[input_name] = self.aggregated_input_data[input_name] + [base]
                print(self.aggregated_input_data)

    def set_up_subscriptions(self, topic_dict: dict):
        """
        Sets up Subscriptions to topics that are mentioned in topic_dict, 
        also sets up listener_callback with correct input-message field mapping

        Args:
            topic_dict (dict): dict that contains the topics to be subscribed to,
            the inputs they carry and in which field these inputs are
        """

        for topic_name in topic_dict:
            msg_type = self.supported_message_types[topic_dict[topic_name]['MessageType']]
            self.create_subscription(msg_type,
                topic_name,
                lambda msg, field_names=topic_dict[topic_name] : self.listener_callback(msg, field_names),
                10)
            self.get_logger().info(f"Subbed to {topic_name}")

    def execute(self):
        """
        Function to execute on a timer. To be filled in child classes.
        """
        return
