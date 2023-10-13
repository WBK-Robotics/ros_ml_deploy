import onnxruntime
import numpy as np

# Main processing function calling a model with given inputs and returning a dict as output
def processing_function(input_dict: dict, model_path: str) -> dict:
    # Create one list to input into the model
    input_list = []
    # Fill the input list with data 
    for key in input_dict.keys():
        input_list += input_dict[key]
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

    #TODO: Add return that makes sense