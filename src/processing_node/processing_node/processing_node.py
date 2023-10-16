import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
import yaml
import os
from typing import TypedDict


class ProcessingNode(Node):
    def __init__(self, func, parameter_file=None):
        """
        Initializes the ProcessingNode with a given function.

        Args:
            func (function): Function with type annotations to set as parameters.
            parameter_file (str, optional): Path to a file where parameters will be saved.
                If provided, the node will listen to parameter events and save changes.
        """
        if not callable(func):
            raise ValueError("`func` must be a callable function.")

        super().__init__('dynamic_parameter_setter')

        self.function_to_execute = func
        self._declare_parameters_for_function(func)

        if parameter_file is not None:
            if not isinstance(parameter_file, str):
                raise ValueError("`parameter_file` must be a string.")
            if not os.path.exists(parameter_file) or not os.access(parameter_file, os.W_OK):
                raise ValueError(f"Unable to access the file {parameter_file} for writing.")

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
        if not isinstance(file_path, str):
            self.get_logger().error("`file_path` must be a string.")
            return

        if not os.path.exists(file_path) or not os.access(file_path, os.W_OK):
            self.get_logger().error(f"Unable to access the file {file_path} for writing.")
            return

        params = {param_name: self.get_parameter(param_name).value for
                   param_name in self.declared_parameters()}

        with open(file_path, 'w') as file:
            yaml.dump({"processing_node": {"ros__parameters": params}}, file)


    def _declare_parameters_for_function(self, func):
        """
        Declares ROS2 parameters for this node based on a given function's type annotations.

        Args:
            func (function): Function with type annotations to set as parameters.
        """
        if "parameters" not in func.__annotations__:
            return

        parameters = func.__annotations__["parameters"]

        for param_name, param_type in parameters.items():
            if param_name in self.declared_parameters():
                self.get_logger().warn(f"Parameter {param_name} is already declared.")
                continue
            self.declare_parameter(param_name)

    def call_function_with_current_parameters(self,func_input):
        """
        Calls the internally stored function using the currently set parameters.

        Returns:
            Result of the function call, or None if no function was set.
        """
        if self.function_to_execute:
            params = {}
            annotations = self.function_to_execute.__annotations__

            for param_name in annotations["parameters"]:
                if not self.has_parameter(param_name):
                    self.get_logger().error(f"Parameter {param_name} is not declared.")
                    return None
            if "parameters" in annotations:
                for param_name in annotations["parameters"]:
                    params[param_name] = self.get_parameter(param_name).value
            return self.function_to_execute(func_input,**params)
        else:
            self.get_logger().warn("No function set to execute!")
            return None


TestTypeDict = {"str parameter": str, "int parameter": int}


def some_function_to_test(main_value:int,parameters: TestTypeDict):
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
