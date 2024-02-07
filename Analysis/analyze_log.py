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

    time = []
    algo_params = ''
    signal = []
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
            elif 'Signal:' in line:
                s = signal_from_line(line)
                if s != 'None':
                    time.append(time_from_line(line))
                    signal.append(float(s))
    return {'Speed': speed, 'Acceleration': acc, 'Algo_params': algo_params, 'Time': time, 'Signal': signal}


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
    plt.plot(data['Time'], data['Signal'])
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('System Response')
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.axhline(y=max(data['Signal']), color='black', linestyle=':')
    plt.axhline(y=max(data['Signal'])-steady_state_error, color='black', linestyle=':')
    plt.axvline(x=data['Time'][0]+timedelta(seconds=convergence_time), color='black', linestyle='--')
    plt.savefig(f'{fig_name}.jpg')


def save_params(data: dict, filename, convergence_time, steady_state_error):
    with open(f'{filename}.txt', 'w') as params_file:
        for key, value in data.items():
            if key in ['Time', 'Signal']:
                continue
            params_file.write(f"{key}: {value}\n")
        params_file.write(f"Convergence time: {convergence_time} [sec]\n")
        params_file.write(f"Steady-State Error: {steady_state_error} [dB]")


if __name__ == '__main__':
    for filename in ['2.log']:
        data = file_to_dict(filename)
        conv_time, ss_error = calc_performance(data)
        save_plot(data, filename.split('.')[0], conv_time, ss_error)
        save_params(data, filename.split('.')[0], conv_time, ss_error)
