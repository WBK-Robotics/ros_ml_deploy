import rclpy
from rclpy.node import Node


class DynamicParameterSetter(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_setter')
        self.function_to_execute = None

    def set_function(self, func):
        self.function_to_execute = func
        self.declare_parameters_for_function(func)

    def declare_parameters_for_function(self, func):
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


def some_function_to_test(parameters: dict):
    print(parameters)
    return parameters


def main():
    rclpy.init()
    node = DynamicParameterSetter()
    node.set_function(some_function_to_test)

    # Assume for testing, we set some parameters externally (e.g., using `ros2 param set ...`)
    # or you can manually set them here for testing purposes
    rclpy.spin_once(node)
    node.call_function_with_current_parameters()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
