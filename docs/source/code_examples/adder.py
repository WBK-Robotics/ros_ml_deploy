import rclpy
import sys

from processing_node.processing_node import ProcessingNode

class WeightedAdder():

    def __init__(self):
        self.parameters = {"float_multiplier": float, "int_multiplier": float}
    
    def get_parameters(self):
        return self.parameters

    def set_parameters(self, parameters: dict):
        for key in parameters.keys():
            if key in self.parameters.keys():
                self.parameters[key] = parameters[key]
            else:
                raise ValueError("The parameter {} is not supported by this processor".format(key))
    
    def execute(self, input_dict:dict) -> dict:
        float_number = input_dict["Float number"][-1]
        int_number = input_dict["Int number"][-1]
        weighted_sum = self.parameters["float_multiplier"] * float(float_number) + self.parameters["int_multiplier"]*float(int_number)
        return_dict = {"Weighted sum": weighted_sum}
        return return_dict

def main(args=None):
    rclpy.init(args=args)
    config_path = sys.argv[1]
    adder_object = WeightedAdder()
    adder_node = ProcessingNode(adder_object, config_path, frequency=50, node_handle="adder_node")

    adder_object.set_parameters({"float_multiplier": 1, "int_multiplier": 2})

    rclpy.spin(adder_node)