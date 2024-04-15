import rclpy
import sys

from processing_node.processing_node import ProcessingNode

class WeightedAdder():

    def __init__(self):
        self.parameters = {"First multiplier": float, "Second multiplier": float}
    
    def get_parameters(self):
        return self.parameters

    def set_parameters(self, parameters: dict):
        for key in parameters.keys():
            if key in self.parameters.keys():
                self.parameters[key] = parameters[key]
            else:
                raise ValueError("The parameter {} is not supported by this processor".format(key))
    
    def execute(self, input_dict:dict) -> dict:
        first_number = input_dict["First number"][-1]
        second_number = input_dict["Second number"][-1]
        weighted_sum = self.parameters["First multiplier"] * float(first_number) + self.parameters["Second multiplier"]*float(second_number)
        return_dict = {"Weighted sum": weighted_sum}
        return return_dict

def main(args=None):
    rclpy.init(args=args)
    config_path = sys.argv[1]
    adder_object = WeightedAdder()
    adder_node = ProcessingNode(adder_object, config_path, frequency=50, node_handle="adder_node")

    adder_object.set_parameters({"First multiplier": 1, "Second multiplier": 2})

    rclpy.spin(adder_node)