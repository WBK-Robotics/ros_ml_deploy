import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
import yaml


class ProcessingNode(Node):
    def __init__(self, func, parameter_file=None):
        """
        Initializes the ProcessingNode with a given function.

        Args:
            func (function): Function with type annotations to set as parameters.
            parameter_file (str, optional): Path to a file where parameters will be saved.
                If provided, the node will listen to parameter events and save changes.
        """
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
        """
        Callback function for parameter change events. If the change event is for this node,
        the updated parameters will be saved to the provided file.

        Args:
            msg (ParameterEvent): Message containing parameter change event data.
        """
        # Check if the parameter change event is for this node
        if msg.node == self.get_fully_qualified_name():
            # Save parameters to a YAML file
            self._save_parameters_to_file(self.parameter_file)

    def _save_parameters_to_file(self, file_path):
        """
        Saves the currently set parameters to a YAML file.

        Args:
            file_path (str): Path to the file where parameters should be saved.
        """
        params = {}
        for param_name in self.declared_parameters():
            params[param_name] = self.get_parameter(param_name).value

        with open(file_path, 'w') as file:
            yaml.dump({"dynamic_parameter_setter": {
                      "ros__parameters": params}}, file)

    def _declare_parameters_for_function(self, func):
        """
        Declares ROS2 parameters for this node based on a given function's type annotations.

        Args:
            func (function): Function with type annotations to set as parameters.
        """
        annotations = func.__annotations__
        if "parameters" in annotations:
            param_types = annotations["parameters"]
            for param_name, param_type in param_types.items():
                # You can also set default values if you want
                self.declare_parameter(param_name)

    def call_function_with_current_parameters(self):
        """
        Calls the internally stored function using the currently set parameters.

        Returns:
            Result of the function call, or None if no function was set.
        """
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
    node = ProcessingNode(some_function_to_test)

    while not rclpy.is_shutdown():
        rclpy.spin_once(node)
        node.call_function_with_current_parameters()


if __name__ == "__main__":
    main()
