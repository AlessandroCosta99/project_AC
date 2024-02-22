import torch.nn as nn
import torch.nn.functional as F

class StrawberryPredictor(nn.Module):
    def __init__(self):
        super(StrawberryPredictor, self).__init__()
        self.fc1 = nn.Linear(4, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 2)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x
    
'''data preprosessing
berry_x = []
berry_y = []
d_berry_x = []
d_berry_y = []
otee13 = []
otee14 = []
ee_vel_x = []
ee_vel_y = []
imp_x = []
imp_y = []

for index, file in enumerate(data_files):
    robot = pd.read_csv(file + "/robot_state.csv")
    berry = pd.read_csv(file + "/strawberry_position.csv")
    time = pd.read_csv(file + "/times.csv")

    ot13 = robot['O_T_EE13']
    ot14 = robot['O_T_EE14']
    fr_x = robot['impedance_force_x']
    fr_y = robot['impedance_force_y']
    berryx = berry['berry_x']
    berryy = berry['berry_y']
    time_values = time['time']

    # Forward difference for all rows except the last
    for i in range(len(berry) - 1):
        vel_x = ot13[i + 1] - ot13[i]
        vel_y = ot14[i + 1] - ot14[i]
        delta_berry_x = berryx[i + 1] - berryx[i]
        delta_berry_y = berryy[i + 1] - berryy[i]
        delta_time = time_values[i + 1] - time_values[i]

        otee13.append(ot13.iloc[i])
        otee14.append(ot14.iloc[i])
        ee_vel_x.append(vel_x / delta_time)
        ee_vel_y.append(vel_y / delta_time)
        imp_x.append(fr_x.iloc[i])
        imp_y.append(fr_y.iloc[i])
        berry_x.append(berryx.iloc[i])
        berry_y.append(berryy.iloc[i])
        d_berry_x.append(delta_berry_x / delta_time)
        d_berry_y.append(delta_berry_y / delta_time)

position_x = np.array(otee13).reshape(-1, 1)
position_y = np.array(otee14).reshape(-1, 1)
velocity_x = np.array(ee_vel_x).reshape(-1, 1)
velocity_y = np.array(ee_vel_y).reshape(-1, 1)
impedance_force_x = np.array(imp_x).reshape(-1, 1)
impedance_force_y = np.array(imp_y).reshape(-1, 1)
berry_x_array = np.array(berry_x).reshape(-1, 1)
berry_y_array = np.array(berry_y).reshape(-1, 1)
d_beery_x_array = np.array(d_berry_x).reshape(-1, 1)
d_berry_y_array = np.array(d_berry_y).reshape(-1, 1)

robot_data = np.column_stack((position_x, position_y, velocity_x, velocity_y, impedance_force_x, impedance_force_y, d_beery_x_array, d_berry_y_array))
berry_data = np.column_stack((berry_x_array, berry_y_array))
'''