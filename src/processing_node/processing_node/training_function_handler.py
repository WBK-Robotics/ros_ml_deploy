import torch
import torch.nn as nn
import numpy as np

# Create one list to input into the model
input_list = []

trained = False

class Autoencoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(100, 64), 
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 12),
        )

        self.decoder = nn.Sequential(
            nn.Linear(12, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU(),
            nn.Linear(64, 100)
        )


    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded

ParameterDict = {}

def training_function(input_dict: dict, parameters: ParameterDict) -> dict:
    global input_list, trained

    number_of_epochs = 50
    model_path = "/root/mounted_folder/ros_ml_deploy/src/processing_node/processing_node/autoencoder.onnx"

    training_sample_multiplier = 100
    
    # Fill the input list with data 
    for key in input_dict.keys():
        input_list += input_dict[key]
        print(f"Data Gathering Process: {len(input_list)} out of {training_sample_multiplier*10+100}")
    
    if len(input_list) < (training_sample_multiplier-1) * 10 + 100:
        return

    input_list = input_list[:training_sample_multiplier*10+100]

    # Scale Data
    max_input = 1.5
    min_input = -1.5
    input_list = [(element - min_input) / (max_input - min_input) for element in input_list]
    # TODO: Generalize scaling for multiple keys

    #TODO: Idee: Überschneidung der Bereiche in den Daten repräsentieren
    input_list_2 = []
    for i in range(int((len(input_list)-100)/10)):
        input_list_2 += input_list[i*10:(i*10+100)]

    # Reshape Data
    inputData = np.array(input_list_2)
    inputData = np.reshape(inputData, ((training_sample_multiplier-1), 100))

    # Convert Data to torch-compatible format
    training_data = torch.tensor(inputData, dtype=torch.float32)
    data_loader = torch.utils.data.DataLoader(dataset=training_data, batch_size=16, shuffle=True)

    # Model parameters
    model = Autoencoder()
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)

    for epoch in range(number_of_epochs):
            for input in data_loader:
                recon = model(input)
                loss = criterion(recon, input)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            print(f"Epoch {epoch}, Loss {loss.item(): .4f}") 

    if not trained:
        torch.save(model.state_dict(), model_path)
        torch.onnx.export(model, 
                        input, 
                        model_path, 
                        input_names=['input'], 
                        output_names=['output'], 
                        dynamic_axes={'input':{0: 'batch_size'}, 
                                        'output':{0 : 'batch_size'}})
        trained = True

    return 0
    #TODO: Add return that makes sense