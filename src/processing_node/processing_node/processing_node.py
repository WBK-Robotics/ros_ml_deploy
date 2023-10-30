from concurrent.futures import process
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class

import sys
import yaml
import importlib

from .processing_function_handler import processing_function

from std_msgs.msg import Float32

class ProcessingNode(Node):

    def __init__(self):
        super().__init__('ProcessingNode')

        config_path = sys.argv[1]

        config = self.loadConfig(config_path)

        # Create data dict which carries the input data
        self.data_dict = dict.fromkeys(config['Inputs'].keys(), [])

        self.importNeededModules(config)

        self.get_logger().info("Setting up subscriptions")

        topic_dict = self.mapInputNamesToTopics(config)

        self.setUpSubscriptions(topic_dict, config)

        # Set up publisher for the model output
        self.output_publisher = self.create_publisher(Float32, 'loss', 10) 
        # TODO: Make output publisher customizable with config input

        self.get_logger().info("Starting the main processing loop")

        # Get model path from config
        self.model_path = config['ModelInformation']['ModelPath']

        # Get model inference timer period from config
        inference_timer_period = float(config['ModelInformation']['ModelInferenceTimerPeriod'])

        # Create timer that calls the processing function based on a timer period specified in the config
        self.processing_timer = self.create_timer(inference_timer_period, self.timer_callback)
    
    def loadConfig(self, config_path):
        """
        Loads config from path and returns it as a dict

        Args:
            config_path (String): Absolute path to the config file expected to be .yaml
        
        Returns:
            config dict
        """

        try:
            # Get dict of inputs from parameters
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            return config

        except:
            self.get_logger().error("Please specify valid config path")
            rclpy.shutdown()
            return
        
    def mapInputNamesToTopics(self, config):
        """
        Load the actual mappings of input names

        Args:
            config (dict): Config dict that specifies requested inputs and relevant information about those inputs
        
        Returns:
            topic dict specifying which topics to subscribe to and what fields of those topics are carrying which input information
        """

        topic_dict = {}

        for key in config['Inputs']:
            topic = config['Inputs'][key]['topic'][0]
            field = config['Inputs'][key]['topic'][1:]
            if topic not in topic_dict:
                topic_dict[topic] = {key: field}
            else:
                topic_dict[topic][key] = field
        
        return topic_dict
    
    def importNeededModules(self, config):
        """
        Imports Modules specified in config dict, possibly needed for output message types

        Args:
            config (dict): Config dict that specifies which modules to import from where
        """

        try:
            import_dict = config['Extras']['Imports']
            for to_import in import_dict.keys():
                import_list = import_dict[to_import]
                importlib.import_module(import_list[0], import_list[1])
        except:
            pass
    
    def setUpSubscriptions(self, topic_dict, config):
        """
        Sets up Subscriptions to topics that are mentioned in topic_dict, also sets up listener_callback with correct input-message field mapping

        Args:
            topic_dict (dict): dict that contains the topics to be subscribed to, the inputs they carry and in which field these inputs are
            config (dict): dict that contains more information about the inputs
        """

        for topic_name in topic_dict.keys():
            subscribed = False
            while not subscribed:
                # Get all currently publishing topics
                topic_list = self.get_topic_names_and_types()
                # Check if topic we want is in currently publishing topics
                for topic_tuple in topic_list:
                    if topic_name in topic_tuple[0]:
                        # Get msg type 
                        msg_type = get_msg_class(self, topic_tuple[0], include_hidden_topics=True)
                        # Set up subscription
                        self.create_subscription(msg_type, topic_tuple[0], lambda msg, field_names=topic_dict[topic_name] : self.listener_callback(msg, field_names), 10)
                        # Toggle bool
                        subscribed = True
                        self.get_logger().info(f"Subscribed to topic {topic_name} which publishes {[config['Inputs'][i]['Name'] for i in topic_dict[topic_name]]}")

    def listener_callback(self, msg, field_names):
        """
        Function that is called whenever something is published to a subscribed topic and modifies the data dict accordingly

        Args:
            msg (ROS2 Message): Message that triggered the function call
            field_names (dict): Dict that specifies which fields are carrying what input information
        """

        # Read the relevant message field and append it to the relevant list in the data dict
        for input in field_names.keys():
            # Loop over attributes to reach deeper message levels until the actual data is reached
            base = msg
            for i in range(len(field_names[input])):
                attribute = field_names[input][i]
                base = getattr(base, attribute)
            # Does not work with append for whatever reason
            self.data_dict[input] = self.data_dict[input] + [base]
            
    def timer_callback(self):
        """
        Function that is called on a timer and sends collected input data to the processing function aswell as publish the resulting output
        """

        #  Call processing function
        processed_data = processing_function(self.data_dict, self.model_path)

        # Reset data dict
        self.data_dict = dict.fromkeys(self.data_dict.keys(), [])

        # Publish output
        # TODO: Make output customizable
        try:
            output_msg = Float32()
            output_msg.data = float(processed_data)
            self.output_publisher.publish(output_msg)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        
