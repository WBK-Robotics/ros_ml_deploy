import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
import yaml


class ProcessingNode(Node):
    def __init__(self, func, parameter_file=None):
        super().__init__('dynamic_parameter_setter')
        self.function_to_execute = func
        self._declare_parameters_for_function(func)

        if parameter_file is not None:
            self.parameter_event_subscription = self.create_subscription(
                ParameterEvent,
                '/parameter_events',
                self.parameter_callback,
                10
            )
        self.parameter_file = parameter_file

    def parameter_callback(self, msg):
        # Check if the parameter change event is for this node
        if msg.node == self.get_fully_qualified_name():
            # Save parameters to a YAML file
            self._save_parameters_to_file(self.parameter_file)

    def _save_parameters_to_file(self, file_path):
        params = {}
        for param_name in self.declared_parameters():
            params[param_name] = self.get_parameter(param_name).value

        with open(file_path, 'w') as file:
            yaml.dump({"dynamic_parameter_setter": {
                      "ros__parameters": params}}, file)

    def _declare_parameters_for_function(self, func):
        annotations = func.__annotations__
        if "parameters" in annotations:
            param_types = annotations["parameters"]
            for param_name, param_type in param_types.items():
                # You can also set default values if you want
                self.declare_parameter(param_name)

    def call_function_with_current_parameters(self):
        if self.function_to_execute:
            params = {}
            annotations = self.function_to_execute.__annotations__
            if "parameters" in annotations:
                for param_name in annotations["parameters"]:
                    params[param_name] = self.get_parameter(param_name).value
            return self.function_to_execute(**params)
        else:
            self.get_logger().warn("No function set to execute!")
            return None


test_type_dict = {"str parameter": str, "int parameter": int}


def some_function_to_test(parameters: test_type_dict):
    print(parameters)
    return parameters


def main():
    rclpy.init()
    node = ProcessingNode()
    node.set_function(some_function_to_test)

    # Assume for testing, we set some parameters externally (e.g., using `ros2 param set ...`)
    # or you can manually set them here for testing purposes
    rclpy.spin_once(node)
    node.call_function_with_current_parameters()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
