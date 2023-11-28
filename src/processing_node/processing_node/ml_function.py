import warnings



class MLFunction:

    def __init__(self):
        pass

    def get_params(self):
        pass

    def train(self, data):
        pass

    def predict(self, data):
        pass


class ConventionalFunction(MLFunction):

    def __init__(self,function):
        """This class is mostly for test purposes
           and allows to use a conventional function instead of a ML model
        """
        self.function = function
        self.parameters = function.__annotations__["parameters"]
        pass

    def set_params(self,parameters):
        # check that parameters only contains valid keys
        if not all(param in self.parameters for param in parameters.keys()):
            raise ValueError("Invalid parameters for function, got: " +
                             str(parameters.keys())+" but expected: "+str(self.parameters.keys()))
        self.parameters = parameters

    def get_params(self):
        return self.parameters


    def call_function_with_current_parameters(self,data):
        """
        Calls the internally stored function using the currently set parameters.

        Returns:
            Result of the function call, or None if no function was set.
        """

        annotations = self.get_params()

        undeclared_params = [param for param in annotations.get("parameters", [])
                             if not self.has_parameter(param)]
        if undeclared_params:
            params_list = ', '.join(undeclared_params)

            warnings.warn(f"Parameters not declared: {params_list}")
            return None

        if annotations is None:
            warnings.warn("No parameters declared for function.")
            return self.function(data)

        params = self.get_params()

        return self.function(data,parameters=params)

    def train(self, data):
        pass

    def predict(self, data):
        return self.call_function_with_current_parameters(data)



def test_conventional_function():
    TestTypeDict = {"float parameter": float, "int parameter": int}

    def some_function_to_test(main_value:int, parameters: TestTypeDict):
        return parameters

    conventional_function = ConventionalFunction(some_function_to_test)

    return conventional_function.predict(1)


if __name__ == "__main__":
    print(test_conventional_function())

