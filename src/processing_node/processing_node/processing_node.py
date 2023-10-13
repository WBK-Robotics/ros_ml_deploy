import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class

import importlib

class ProcessingNode(Node):

    def __init__(self):
        # Construct Node, automatically discover parameters from yaml file
        super().__init__('ProcessingNode', automatically_declare_parameters_from_overrides = True)

        self.get_logger().info("Loading parameters from yaml")

        # Get dict of inputs from parameters
        self.input_dict = self.get_parameters_by_prefix('Inputs')

        # Load the actual mappings of input names
        for key in self.input_dict.keys():
            self.input_dict[key] = self.get_parameter(f'Inputs.{key}').get_parameter_value().string_array_value

        # Create data dict with the same keys as the input_dict
        self.data_dict = dict.fromkeys(self.input_dict.keys(), [])
        
        # Import custom message types specified in yaml
        # Not sure if needed right now, kept for future proofing (possibly for sending output?)
        try:
            import_dict = self.get_parameters_by_prefix('Extras.Imports')
            for to_import in import_dict.keys():
                import_list = self.get_parameter(f'Extras.Imports.{to_import}').get_parameter_value().string_array_value
                importlib.import_module(import_list[0], import_list[1])
        except:
            pass

        self.get_logger().info("Setting up subscriptions")

        # Create subscription to every topic specified in parameters
        for input_name in self.input_dict.keys():
            subscribed = False
            topic_name = self.input_dict[input_name][0]
            field_name = self.input_dict[input_name][1]
            while not subscribed:
                # Get all currently publishing topics
                topic_list = self.get_topic_names_and_types()
                # Check if topic we want is in currently publishing topics
                for topic_tuple in topic_list:
                    if topic_name in topic_tuple[0]:
                        # Get msg type 
                        msg_type = get_msg_class(self, topic_tuple[0], include_hidden_topics=True)
                        # Set up subscription
                        self.create_subscription(msg_type, topic_tuple[0], lambda msg, field_name=self.input_dict[input_name][1]: self.listener_callback(msg, field_name), 10)
                        # Toggle bool
                        subscribed = True
                        self.get_logger().info(f"Subscribed to topic regarding {input_name}")
    
    def listener_callback(self, msg, field_name):
        print(f"\n{getattr(msg, field_name)}")

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        
