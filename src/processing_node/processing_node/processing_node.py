from concurrent.futures import process
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class

import importlib

from .processing_function_handler import processing_function
from .training_function_handler import training_function

from std_msgs.msg import Float32

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

        # Create sample number dict
        self.sample_number_dict = self.get_parameters_by_prefix('NumberOfInputSamples')

        # Check if training mode or not
        self.training_mode = self.get_parameter('Training.TrainingMode').get_parameter_value().bool_value

        # Get Training Sample Multiplier
        self.training_sample_multiplier = self.get_parameter('Training.TrainingSampleMultiplier').get_parameter_value().integer_value

        # Get Number of Epochs for Training
        self.number_of_epochs = self.get_parameter('Training.NumberOfEpochs').get_parameter_value().integer_value

        # Load the number of samples per input name
        for key in self.sample_number_dict.keys():
            self.sample_number_dict[key] = self.get_parameter(f'NumberOfInputSamples.{key}').get_parameter_value().integer_value
        
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
                        self.create_subscription(msg_type, topic_tuple[0], lambda msg, field_name=field_name, dict_key=input_name : self.listener_callback(msg, field_name, dict_key), 10)
                        # Toggle bool
                        subscribed = True
                        self.get_logger().info(f"Subscribed to topic {topic_name} which publishes {input_name}")

        # Set up publisher for the model output
        self.output_publisher = self.create_publisher(Float32, 'loss', 10) 
        # TODO: Make output publisher customizable with config input

        self.get_logger().info("Starting the main processing loop")

        # Get model path from config
        self.model_path = self.get_parameter('ModelInformation.ModelPath').get_parameter_value().string_value

        # Get model inference timer period from config
        inference_timer_period = self.get_parameter('ModelInformation.ModelInferenceTimerPeriod').get_parameter_value().double_value

        # Create timer that calls the processing function based on a timer period specified in the config
        self.processing_timer = self.create_timer(inference_timer_period, self.timer_callback)
    
    # One-fits-all-function that takes messages and edits the data dict accordingly
    def listener_callback(self, msg, field_name, dict_key):
        # Sanity check if the input name is requested by the config
        if dict_key in self.sample_number_dict:
            # Read the relevant message field and append it to the relevant list in the data dict 
            self.data_dict[dict_key].append(getattr(msg, field_name))
            # Calculate maximum number of samples in the dict
            if self.training_mode:
                maximum_sample_number = self.sample_number_dict[dict_key] * self.training_sample_multiplier
            else:
                maximum_sample_number = self.sample_number_dict[dict_key]
            # Ensure that no more than the latest number of samples are in the dict
            if len(self.data_dict[dict_key]) > maximum_sample_number:
                self.data_dict[dict_key].pop(0)
            
    def timer_callback(self):
        # Inference Mode
        if not self.training_mode:
            # Check if there is enough data in the data dict to run an inference
            number_of_features_ready = 0
            for key in self.sample_number_dict.keys():
                if not len(self.data_dict[key]) == self.sample_number_dict[key]:
                    self.get_logger().info("Waiting for data")
                else:
                    number_of_features_ready += 1

            if number_of_features_ready == len(self.sample_number_dict.keys()):
                #  Call processing function
                processed_data = processing_function(self.data_dict, self.model_path)

                # Publish output
                # TODO: Make output customizable
                output_msg = Float32()
                output_msg.data = float(processed_data)
                self.output_publisher.publish(output_msg)
            

        # Training Mode
        else:
            # Check if there is enough data in the data dict to run training
            number_of_features_ready = 0
            for key in self.sample_number_dict.keys():
                if not len(self.data_dict[key]) == self.sample_number_dict[key] * self.training_sample_multiplier:
                    self.get_logger().info(f"Waiting for data, {len(self.data_dict[key])} of {self.sample_number_dict[key] * self.training_sample_multiplier}")
                else:
                    number_of_features_ready += 1

            # Start training once enough data is collected
            if number_of_features_ready == len(self.sample_number_dict.keys()):
                training_function(self.data_dict, self.model_path, self.training_sample_multiplier, self.number_of_epochs)
                self.training_mode = False

    # TODO: Make the processing function more generic

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        
