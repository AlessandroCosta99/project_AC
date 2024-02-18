import torch
import torch.nn as nn

class SequencePredictor(nn.Module):
    def __init__(self, input_size, hidden_size, output_size, num_layers):
        super(SequencePredictor, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        self.num_layers = num_layers

        # LSTM layer
        self.lstm = nn.LSTM(input_size=input_size, 
                            hidden_size=hidden_size, 
                            num_layers=num_layers, 
                            batch_first=True)

        # Fully connected layer
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        # Hidden state and cell state
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)

        # Forward propagate the LSTM
        out, _ = self.lstm(x, (h0, c0))
        
        # Pass the output of the last time step through the fully connected layer
        out = self.fc(out[:, -1, :])
        
        out = out.view(-1, 10, 2)

        return out