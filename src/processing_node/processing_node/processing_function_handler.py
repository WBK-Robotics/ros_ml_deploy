from sys import intern
import onnxruntime
import numpy as np

import torch
import torch.nn as nn

# Create one list to input into the model
input_list = []

# Main processing function calling a model with given inputs and returning a dict as output
def processing_function(input_dict: dict, model_path: str) -> dict:
    global input_list
    # Fill the input list with data 
    for key in input_dict.keys():
        input_list += input_dict[key]
    
    if len(input_list) < 100:
        return

    if len(input_list) > 100:
        input_list = input_list[len(input_list)-100:]

    # Scale Data
    max_input = max(input_list)
    min_input = min(input_list)
    input_list = [(element - min_input) / (max_input - min_input) for element in input_list]

    input_array = np.array(input_list, dtype=np.float32)
    input_array = np.reshape(input_array, (1, -1))

    ort_session = onnxruntime.InferenceSession(model_path, providers=['CPUExecutionProvider'])

    try:
        ort_input = {'input': input_array}
        recon = ort_session.run(None, ort_input)
    except:
        ort_input = {'input': input_array}
        recon = ort_session.run(None, ort_input)
    
    loss = np.mean((recon-input_array)**2, axis=2)

    loss = loss[0][0]
    
    return loss

