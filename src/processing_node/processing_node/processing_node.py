import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
import yaml
import os

class ProcessingNode(Node):
    def __init__(self, func,frequency=30):
        """
        Initializes the ProcessingNode with a given function.

        Args:
            func (function): Function with type annotations to set as parameters.
            frequency (float): Frequency at which the function should be called.
        """
        if not callable(func):
            raise ValueError("`func` must be a callable function.")

        super().__init__('processing_node')

        self.function_to_execute = func
        self.declared_parameters = []
        self._declare_parameters_for_function(func)

        timer_period = 1.0 / frequency
        self.timer = self.create_timer(timer_period, self.execute_function)


    def _declare_parameters_for_function(self, func):
        """
        Declares ROS2 parameters for this node based on a given function's type annotations.

        Args:
            func (function): Function with type annotations to set as parameters.
        """
        if "parameters" not in func.__annotations__:
            return

        parameters = func.__annotations__["parameters"]
        rclpy.logging.get_logger("processing_node").info(f"Parameters: {parameters}")
        for param_name, param_type  in parameters.items():
            if param_type not in [int, float, str, bool]:
                self.get_logger().error(f"Parameter type {param_type} is not supported.")
                continue
            if self.has_parameter(param_name):
                self.get_logger().warn(f"Parameter {param_name} is already declared.")
                continue

            if param_type == int:
                self.declare_parameter(param_name, 0)
            elif param_type == float:
                self.declare_parameter(param_name, 0.0)
            elif param_type == str:
                self.declare_parameter(param_name, "")
            elif param_type == bool:
                self.declare_parameter(param_name, False)

            self.declared_parameters.append(param_name)


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
            return self.function_to_execute(func_input,parameters=params)
        else:
            self.get_logger().warn("No function set to execute!")
            return None


    def execute_function(self):
        """
        This callback is triggered by the ROS timer.
        """
        func_input = 10  # Example value, modify as needed
        result = self.call_function_with_current_parameters(func_input)
        self.get_logger().info(f"Function result: {result}")


TestTypeDict = {"float parameter": float, "int parameter": int}


def some_function_to_test(main_value:int,parameters: TestTypeDict):
    print(parameters)
    return parameters


def main():
    rclpy.init()
    node = ProcessingNode(some_function_to_test)

    rclpy.spin(node)


if __name__ == "__main__":

    main()
