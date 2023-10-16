import rclpy
from rclpy.node import Node


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
            if self.has_parameter(param_name):
                self.get_logger().warn(f"Parameter {param_name} is already declared.")
                continue

            default_values = {int: 0,
                              float: 0.0,
                              str: "",
                              bool: False}

            if param_type not in default_values.keys():
                self.get_logger().error(f"Parameter type {param_type} is not supported.")
                continue

            self.declare_parameter(param_name, default_values[param_type])


    def call_function_with_current_parameters(self,func_input):
        """
        Calls the internally stored function using the currently set parameters.

        Returns:
            Result of the function call, or None if no function was set.
        """

        annotations = self.function_to_execute.__annotations__

        if not self.function_to_execute:
            self.get_logger().warn("No function set to execute!")
            return None

        undeclared_params = [param for param in annotations.get("parameters", [])
                             if not self.has_parameter(param)]
        if undeclared_params:
            params_list = ', '.join(undeclared_params)
            self.get_logger().error(f"Parameters not declared: {params_list}")
            return None

        if "paramters" not in annotations:
            self.get_logger().warn("No parameters declared for function.")
            return self.function_to_execute(func_input)

        params = {}
        for param_name in annotations["parameters"]:
            params[param_name] = self.get_parameter(param_name).value
        return self.function_to_execute(func_input,parameters=params)


    def execute_function(self):
        """
        This callback is triggered by the ROS timer. It calls the internally stored function.

        TODO this is a placeholder to be latter married with marvins code wherin the function inputs
        are given by the configured subscribers and the function output is published
        to the configured publishers.
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
