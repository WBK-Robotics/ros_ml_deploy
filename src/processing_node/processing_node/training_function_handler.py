import torch
import torch.nn as nn
import numpy as np

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

def training_function(input_dict: dict, model_path: str, training_sample_multiplier: int, number_of_epochs: int) -> dict:
    # Create one list to input into the model
    input_list = []
    # Fill the input list with data 
    for key in input_dict.keys():
        input_list += input_dict[key]

    # Scale Data
    max_input = max(input_list)
    min_input = min(input_list)
    input_list = [(element - min_input) / (max_input - min_input) for element in input_list]
    # TODO: Generalize scaling for multiple keys

    # Reshape Data
    inputData = np.array(input_list)
    inputData = np.reshape(inputData, (training_sample_multiplier, int(len(input_list)/training_sample_multiplier)))

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

    torch.save(model.state_dict(), model_path)
    torch.onnx.export(model, 
                    input, 
                    model_path, 
                    input_names=['input'], 
                    output_names=['output'], 
                    dynamic_axes={'input':{0: 'batch_size'}, 
                                    'output':{0 : 'batch_size'}})

    return 0
    #TODO: Add return that makes sense