import socket
import subprocess
import re
from datetime import datetime

HOST = "192.168.200.211"
PORT = 65432


def parse_status(status: str):
    match = re.search('Position uncertain:\s*', status)
    position_uncertain = True if status[match.end()] == 'Y' else False
    match = re.search(r'Current position:\s+-?\d+\n', status)
    match = re.search(r'-?\d+', match.group(0))
    current_position = int(match.group(0))
    return position_uncertain, current_position


# print(parse_status(b'Name:                         Tic 36v4 High-Power Stepper Motor Controller\nSerial number:                00314754\nFirmware version:             1.07\nLast reset:                   Power-on reset\nUp time:                      6:28:07\n\nEncoder position:             0\nRC pulse width:               N/A\nInput state:                  Halt\nInput after averaging:        N/A\nInput after hysteresis:       N/A\nInput before scaling:         N/A\nInput after scaling:          0\nForward limit active:         No\nReverse limit active:         No\n\nVIN voltage:                  28.021 V\nOperation state:              Normal\nEnergized:                    Yes\nHoming active:                No\n\nTarget:                       No target\nCurrent position:             0\nPosition uncertain:           No\nCurrent velocity:             0\nMax speed:                    20000000\nStarting speed:               0\nMax acceleration:             1000000\nMax deceleration:             1000000\nActing target position:       0\nTime since last step:         45000\nStep mode:                    1/4 step\nCurrent limit:                2793 mA\n\nErrors currently stopping the motor: None\nErrors that occurred since last check: None\nLast motor driver errors:\n  - Undervoltage\n\nSCL pin:\n  State:                      Output low\n  Analog reading:             N/A\n  Digital reading:            1\nSDA pin:\n  State:                      Output low\n  Analog reading:             N/A\n  Digital reading:            1\nTX pin:\n  State:                      Output high\n  Analog reading:             N/A\n  Digital reading:            1\nRX pin:\n  State:                      Pulled up\n  Analog reading:             N/A\n  Digital reading:            1\nRC pin:\n  Digital reading:            1\n\n'.decode()))

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        print("Connected by ", addr)
        connection_status = {"status": "open"}
        with conn:
            while True:
                try:
                    data = conn.recv(1024).decode().split("\n")
                except:
                    break
                if data == ['']:
                    continue
                for d in data:
                    if d == 'exit':
                        print(addr, "Disconnected")
                        connection_status["status"] = "close"
                        break
                    elif d == '':
                        continue
                    else:
                        pytic_output = subprocess.check_output(eval(d)).decode()
                        if pytic_output != '':
                            # print(str(parse_status(pytic_output)))
                            conn.sendall(str(parse_status(pytic_output)).encode())
                if connection_status["status"] == "close":
                    break
