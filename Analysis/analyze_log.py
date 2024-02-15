import os
import numpy as np
import re
from datetime import datetime, timedelta
import matplotlib.pyplot as plt


def file_to_dict(file_name):
    def time_from_line(line):
        t = ' '.join(line.split()[:2])
        return datetime.strptime(t, '%Y/%m/%d %H:%M:%S')

    def tuple_from_line(line):
        match = re.search(r"\((.*?)\)", line)
        return match.group()

    def algo_params_from_line(line):
        return ','.join(line.split('=')[1:])[:-1]

    def time_between_nodes_from_line(line):
        return line.split('=')[1]

    def signal_from_line(line):
        return line.split('Signal: ')[1].split('\n')[0]

    def positioner_from_line(line):
        return eval(tuple_from_line(line.split('Positioner: ')[1]))

    def gimbal_from_line(line):
        return eval(tuple_from_line(line.split('Gimbal: ')[1]))

    time = []
    algo_params = ''
    signal = []
    positioner_pos = []
    gimbal_pos = []
    with open(file_name, 'r') as log:
        for line in log.readlines():
            if 'Gimbal speed set to:' in line:
                speed = eval(tuple_from_line(line))
            elif 'Gimbal acceleration set to:' in line:
                acc = eval(tuple_from_line(line))
            elif 'Algo=' in line:
                algo_params = (algo_params_from_line(line))
            elif 'time_between_nodes' in line:
                algo_params = algo_params + f',time_between_nodes={time_between_nodes_from_line(line)}'
            elif 'Signal:' in line and 'None' not in line:
                s = signal_from_line(line)
                time.append(time_from_line(line))
                signal.append(float(s))
                p = positioner_from_line(line)
                positioner_pos.append(p)
                g = gimbal_from_line(line)
                gimbal_pos.append(g)

    return {'Speed': speed, 'Acceleration': acc, 'Algo_params': algo_params, 'Time': time, 'Signal': signal, 'Positioner': positioner_pos, 'Gimbal': gimbal_pos}


def calc_performance(data: dict):
    max_signal = max(data['Signal'])
    threshold = max_signal / (max_signal-1)  # 1 dB from max
    convergence_time = data['Signal'][-1]
    convergence_time_index = 0
    for convergence_time_index, convergence_time in enumerate(data['Time']):
        if data['Signal'][convergence_time_index] >= (1-threshold)*max_signal+max_signal:
            break
    convergence_time = (convergence_time-data['Time'][0]).total_seconds()

    steady_state_error = round(max_signal - min(data['Signal'][convergence_time_index:]), 1)
    return convergence_time, steady_state_error


def save_plot(data: dict, fig_name, convergence_time, steady_state_error):
    plt.figure()
    plt.plot(data['Time'], data['Signal'])
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('System Response')
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.axhline(y=max(data['Signal']), color='black', linestyle=':')
    plt.axhline(y=max(data['Signal'])-steady_state_error, color='black', linestyle=':', label=f'SS error: {steady_state_error} [dB]')
    plt.axvline(x=data['Time'][0]+timedelta(seconds=convergence_time), color='black', linestyle='--', label=f'Convergence time: {convergence_time} [sec]')
    plt.legend()
    plt.savefig(f'{fig_name}.jpg')


def save_plot_position(data, fig_name):
    plt.figure()
    cmap = plt.get_cmap('inferno')
    plt.scatter([pos[0] for pos in data['Positioner']], [pos[1] for pos in data['Positioner']], color='black', label='Positioner')
    x = [pos[0] for pos in data['Gimbal']]
    y = [pos[1] for pos in data['Gimbal']]
    s = data['Signal']
    sn = [abs(a) for a in s]
    for i in range(len(x) - 1):
        if i == 0:
            plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], color=cmap(1 - ((sn[i]-min(sn)) / (max(sn)-min(sn)))), label='Gimbal')
        else:
            plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], color=cmap(1 - ((sn[i]-min(sn)) / (max(sn)-min(sn)))))
    # plt.plot([pos[0] for pos in data['Gimbal']], [pos[1] for pos in data['Gimbal']], color='green', linestyle='solid', label='Gimbal')
    plt.legend()
    plt.grid()
    plt.savefig(f'{fig_name}_positions.jpg')


def save_params(data: dict, filename, convergence_time, steady_state_error):
    with open(f'{filename}.txt', 'w') as params_file:
        for key, value in data.items():
            if key in ['Time', 'Signal']:
                continue
            params_file.write(f"{key}: {value}\n")
        params_file.write(f"Convergence time: {convergence_time} [sec]\n")
        params_file.write(f"Steady-State Error: {steady_state_error} [dB]")


def process_log(filename):
    data = file_to_dict(filename)
    conv_time, ss_error = calc_performance(data)
    save_plot(data, filename.split('.')[0], conv_time, ss_error)
    save_plot_position(data, filename.split('.')[0])
    save_params(data, filename.split('.')[0], conv_time, ss_error)


if __name__ == '__main__':
    for filename in os.listdir():
        if not filename.split(".")[0].isnumeric() or filename.split(".")[1] != "log":
            continue
        process_log(filename)
