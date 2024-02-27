# -*- coding: utf-8 -*-
# RUN IN PYTHON 3

import torch
import torch.nn as nn
from itertools import cycle


class ConvLSTMCell(nn.Module):
    def __init__(self, input_dim, hidden_dim, kernel_size, bias, device):
        super(ConvLSTMCell, self).__init__()
        self.input_dim = input_dim
        self.device = device
        self.hidden_dim = hidden_dim
        self.kernel_size = kernel_size
        self.padding = kernel_size[0] // 2, kernel_size[1] // 2
        self.bias = bias
        self.conv = nn.Conv2d(in_channels=self.input_dim + self.hidden_dim, out_channels=4 * self.hidden_dim,
                              kernel_size=self.kernel_size, padding=self.padding, bias=self.bias)

    def forward(self, input_tensor, cur_state):
        h_cur, c_cur = cur_state
        combined = torch.cat([input_tensor, h_cur], dim=1)  # concatenate along channel axis
        combined_conv = self.conv(combined)
        # cc_i, cc_f, cc_o, cc_g = torch.split(combined_conv, self.hidden_dim, dim=1)
        cc_i, cc_f, cc_o, cc_g = combined_conv[:, :self.hidden_dim], combined_conv[:, self.hidden_dim:2*self.hidden_dim],\
            combined_conv[:, 2*self.hidden_dim:3*self.hidden_dim], combined_conv[:, 3*self.hidden_dim: 4*self.hidden_dim]
        i = torch.sigmoid(cc_i)
        f = torch.sigmoid(cc_f)
        o = torch.sigmoid(cc_o)
        g = torch.tanh(cc_g)
        c_next = f * c_cur + i * g
        h_next = o * torch.tanh(c_next)
        return h_next, c_next

    def init_hidden(self, batch_size, image_size):
        height, width = image_size
        return (torch.zeros(batch_size, self.hidden_dim, height, width, device=self.conv.weight.device),
                torch.zeros(batch_size, self.hidden_dim, height, width, device=self.conv.weight.device))

class Model(nn.Module):
    def __init__(self, features):
        super(Model, self).__init__()
        self.features = features
        self.device = features["device"]
        self.context_frames = features["n_past"]
        self.n_future = features["n_future"]
        self.model_dir = features["model_dir"]
        self.model_name_save_appendix = features["model_name_save_appendix"]
        self.convlstm1 = ConvLSTMCell(input_dim=128, hidden_dim=128, kernel_size=(3, 3), bias=True, device=self.device)
        self.convlstm2 = ConvLSTMCell(input_dim=140, hidden_dim=140, kernel_size=(3, 3), bias=True, device=self.device)
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=64, kernel_size=5, stride=1, padding=2)
        self.conv12 = nn.Conv2d(in_channels=64, out_channels=128, kernel_size=5, stride=1, padding=2)
        self.conv2 = nn.Conv2d(in_channels=6, out_channels=3, kernel_size=5, stride=1, padding=2)
        self.upconv1 = nn.Conv2d(in_channels=140, out_channels=64, kernel_size=5, stride=1, padding=2)
        self.upconv2 = nn.Conv2d(in_channels=64, out_channels=3, kernel_size=5, stride=1, padding=2)
        self.maxpool1 = nn.MaxPool2d(2, stride=2)
        self.relu1 = nn.ReLU()
        self.upsample2 = nn.Upsample(scale_factor=2)
        self.tanh = nn.Tanh()

    def forward(self, actions, touch, test=False):
       
        self.batch_size = actions.shape[1]  # 1
        state = actions[0]  # 0
        batch_size__ = touch.shape[1]  # 1
        hidden_1, cell_1 = self.convlstm1.init_hidden(batch_size=self.batch_size, image_size=(16, 16))
        hidden_2, cell_2 = self.convlstm2.init_hidden(batch_size=self.batch_size, image_size=(16, 16))
        # Initialize output
        outputs = []
        for index, (sample_action, sample_touch) in enumerate(zip(actions[1:], cycle(touch[:-1]))):
            # 2. Run through lstm:
            if index > self.context_frames - 1:
                # DOWNSAMPLING
                # Touch Downsampling
                out_touch1 = self.maxpool1(self.relu1(self.conv1(output)))
                out_touch4 = self.maxpool1(self.relu1(self.conv12(out_touch1)))

                state_action = torch.cat((state, sample_action), 1)

                # LSTM Chain
                hidden_1, cell_1 = self.convlstm1(input_tensor=out_touch4, cur_state=[hidden_1, cell_1])
                robot_and_touch = torch.cat((torch.cat(
                    16 * [torch.cat(16 * [state_action.unsqueeze(2)], axis=2).unsqueeze(3)], axis=3), hidden_1), 1)
                hidden_2, cell_2 = self.convlstm2(input_tensor=robot_and_touch, cur_state=[hidden_2, cell_2])
                
                # UPSAMPLING
                up1 = self.upsample2(self.relu1(self.upconv1(hidden_2)))
                up4 = self.upsample2(self.relu1(self.upconv2(up1)))

                skip_connection_added = torch.cat((up4, output), 1)
                output = self.conv2(skip_connection_added)
                output = self.tanh(output)

                outputs.append(output)

            else:
                # Touch Downsampling
                out_touch1 = self.maxpool1(self.relu1(self.conv1(sample_touch)))
                out_touch4 = self.maxpool1(self.relu1(self.conv12(out_touch1)))

                state_action = torch.cat((state, sample_action), 1)

                # LSTM Chain
                hidden_1, cell_1 = self.convlstm1(input_tensor=out_touch4, cur_state=[hidden_1, cell_1])
                robot_and_touch = torch.cat((torch.cat(
                    16 * [torch.cat(16 * [state_action.unsqueeze(2)], axis=2).unsqueeze(3)], axis=3), hidden_1), 1)
                hidden_2, cell_2 = self.convlstm2(input_tensor=robot_and_touch, cur_state=[hidden_2, cell_2])
                # UPSAMPLING
                up1 = self.upsample2(self.relu1(self.upconv1(hidden_2)))
                up4 = self.upsample2(self.relu1(self.upconv2(up1)))

                skip_connection_added = torch.cat((up4, sample_touch), 1)
                output = self.conv2(skip_connection_added)

                last_output = output

        outputs = [last_output] + outputs

        return torch.stack(outputs)


