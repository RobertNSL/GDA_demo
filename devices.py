import math
import gclib
import asyncio
import pandas as pd
import vxi11
from PyTic import *
from datetime import datetime, timedelta
import torch
import torch.optim as optim
import numpy as np
from scipy.signal import butter


class Newmark:
    def __init__(self, COM, baud):
        self.COM = COM
        self.baud = baud
        self.Az_steps_in_deg = 12500
        self.Az_encoder_counts_deg = 1000
        self.El_encoder_counts_deg = 1000
        self.El_steps_in_deg = 12500
        self.position = None

    async def connect(self):
        self.g = gclib.py()
        self.g.GOpen(f'{self.COM} --baud {self.baud}')
        logger.info(f"Newmark is connected at {self.COM}")
        self.g.GCommand('PT 1,1')  # switches to tracking mode (non-blocking)
        asyncio.create_task(self.read_position(0.2))

    async def disconnect(self):
        logger.info("Newmark disconnected")
        self.g.GClose()
        self.g = None

    async def go_to(self, az, el, blocking=False):
        self.g.GCommand(f'PAA={az*self.Az_steps_in_deg};PAB={el*self.El_steps_in_deg}')
        if blocking:
            self.g.GMotionComplete('AB')

    async def go_to_az(self, az):
        self.g.GCommand(f'PAA={az*self.Az_steps_in_deg}')

    async def go_to_el(self, el):
        self.g.GCommand(f'PAB={el*self.El_steps_in_deg}')

    async def set_speed(self, speed_az, speed_el):  # deg/sec
        self.g.GCommand(f'SPA={speed_az*self.Az_steps_in_deg};SPB={speed_el*self.El_steps_in_deg}')
        logger.info(f"Gimbal speed set to: {speed_az, speed_el} [deg/sec]")

    async def set_acceleration(self, acc_az, acc_el):
        self.g.GCommand(f'ACA={acc_az * self.Az_steps_in_deg};ACB={acc_el * self.El_steps_in_deg};DCA={acc_az * self.Az_steps_in_deg};DCB={acc_el * self.El_steps_in_deg}')
        logger.info(f"Gimbal acceleration set to: {acc_az, acc_el} [deg/sec^2]")

    async def read_position(self, delay_sec):
        while self.g:
            await asyncio.sleep(delay_sec)
            result = self.g.GCommand('TP').split(',')
            az = round(int(result[0])/self.Az_encoder_counts_deg, 1)
            el = round(int(result[1])/self.El_encoder_counts_deg, 1)
            self.position = az, el


class Positioner:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.az_controller = TicController(serialnumber='00314754', current_limit=2793, max_accel=1000000, max_deccel=1000000, max_speed=20000000, step_size=StepSizes.ONEQUARTER)
        self.el_controller = TicController(serialnumber='00305902', current_limit=2793, max_accel=1000000, max_deccel=1000000, max_speed=20000000, step_size=StepSizes.ONEQUARTER)
        self.position = None
        self.reader, self.writer = None, None
        self.zero_position = (-59, 7.8)

    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(self.ip, self.port)
        asyncio.create_task(self.update_position(0.5))
        asyncio.create_task(self.Tic_reset_timeout())
        logger.info("Positioner is connected")

    async def Tic_reset_timeout(self):
        while self.writer:
            await asyncio.sleep(0.8)
            await self._send_cmd(self.az_controller.reset_command_timeout())
            await self._send_cmd(self.el_controller.reset_command_timeout())

    async def disconnect(self):
        await self._send_cmd("exit")
        self.reader = None
        self.writer.close()
        await self.writer.wait_closed()
        logger.info("Positioner disconnected")

    async def _send_cmd(self, cmd: str):
        self.writer.write(f'{cmd}\n'.encode())
        await self.writer.drain()

    def _az_to_steps(self, deg):
        AZ_Zero = 90
        azcal = 1 / (53.33 * 4)
        return int((deg-AZ_Zero)/azcal)

    def _steps_to_az(self, steps):
        AZ_Zero = 90
        azcal = 1 / (53.33 * 4)
        return steps*azcal+AZ_Zero

    def _el_to_steps(self, deg):
        MaxElevation = 18.8
        elcal=-13.9/(1000*4)
        return int((deg-MaxElevation)/elcal)

    def _steps_to_el(self, steps):
        MaxElevation = 18.8
        elcal=-13.9/(1000*4)
        return steps*elcal+MaxElevation

    async def set_speed(self, az_deg_sec, el_deg_sec):
        await self._send_cmd(self.az_controller._set_max_velocity(int(abs(az_deg_sec)*(53.33*4)) * 10000))
        await self._send_cmd(self.el_controller._set_max_velocity(int(abs(el_deg_sec)*(4000/13.9)) * 10000))

    async def go_to(self, az, el):
        await self._send_cmd(self.az_controller.move(self._az_to_steps(az+self.zero_position[0])))
        await self._send_cmd(self.el_controller.move(self._el_to_steps(el+self.zero_position[1])))

    async def go_to_at_speed(self, az, el, az_deg_sec, el_deg_sec):
        await self.set_speed(az_deg_sec, el_deg_sec)
        await self.go_to(az, el)

    async def get_position(self):
        if self.reader is None:
            return
        await self._send_cmd(self.az_controller.get_pos())
        rec = await self.reader.read(100)
        try:
            rec = eval(rec.decode())
        except:
            return
        az_position_uncertain = rec[0]
        if az_position_uncertain:
            await self._send_cmd(self.az_controller.perform_homing(dir=1))
        az_pos = self._steps_to_az(rec[1])

        await self._send_cmd(self.el_controller.get_pos())
        rec = await self.reader.read(100)
        rec = eval(rec.decode())
        el_position_uncertain = rec[0]
        if el_position_uncertain:
            await self._send_cmd(self.el_controller.perform_homing(dir=0))
        el_pos = self._steps_to_el(rec[1])
        self.position = (round(az_pos-self.zero_position[0], 1), round(el_pos-self.zero_position[1], 1))

    async def update_position(self, delay_sec):
        while True:
            await asyncio.sleep(delay_sec)
            await self.get_position()


class Network_Analizer:
    def __init__(self, ip, start_freq, measure_freq, stop_freq):
        self.ip = ip
        self.instr = None
        self.RSSI = None
        self.start_freq = start_freq
        self.stop_freq = stop_freq
        self.measure_freq = measure_freq

    async def connect(self):
        self.instr = vxi11.Instrument(self.ip)
        self.instr.timeout = 2
        spectrum_name = self.instr.ask("*IDN?")
        logger.info(f"Network is connected - {spectrum_name}")
        await self.config()
        logger.info("Network configed")
        asyncio.create_task(self.update_RSSI())

    async def disconnect(self):
        if self.instr is None:
            return

        self.instr.close()
        self.instr = None
        logger.info("Network is disconnected")

    async def config(self):
        # print('Setting S12 masurment, from', self.start_freq, 'to', self.stop_freq)
        self.instr.write('*RST')
        self.instr.write(':DISP:WIND1:TRAC1:DEL')
        self.instr.write(':CALC:PAR:DEF:EXT "s12_1","S12"')
        self.instr.write(':CALC:PAR:SEL "s12_1"')
        self.instr.write(':DISP:WIND:TRAC1:FEED "s12_1"')
        self.instr.write(':SENS:FREQ:STAR ' + str(self.start_freq) + ' GHZ')
        self.instr.write(':SENS:FREQ:STOP ' + str(self.stop_freq) + ' GHZ')
        self.instr.write('SENS:AVER ON')
        self.instr.write('SENS:AVER:COUN 5')
        self.instr.write(':DISP:WIND1:TRAC1:Y:RLEV -50')
        self.instr.write(':CALC:MARK1:STAT ON;X ' + str(self.measure_freq) + ' GHz')

    async def read(self):
        if self.instr is None:
            return

        buff = self.instr.ask(":CALC:MARK1:y?")
        amp = buff.split(',')[0]
        self.RSSI = float("{:05.1f}".format(float(amp)))

    async def update_RSSI(self):
        while self.instr:
            await asyncio.sleep(0.05)
            await self.read()


class System:
    def __init__(self, mode, positioner, gimbal, signal):
        self.start_time = None
        self.modes = ["idle", "track_signal_discrete", 'track_signal_SGD', 'track_signal_ESC', "search", "track_trajectory"]
        self.threshold = -58
        self.mode = mode
        self.active_mode_task = None
        self.positioner = positioner
        self.gimbal = gimbal
        self.signal = signal
        self.trajectory = pd.read_csv("positioner_trajectory.csv")
        self.next_trajectory_position = None

    def positioner_to_gimbal_axis(self, positioner_position):  # TODO: change to generalized quaternion axes conversion
        gimbal_az = -positioner_position[0]
        gimbal_el = -positioner_position[1]
        return gimbal_az, gimbal_el

    async def mode_manager(self):
        while True:
            if self.signal.RSSI > self.threshold and self.mode != "track_signal_ESC":
                await self.set_mode("track_signal_ESC")
            elif self.signal.RSSI < self.threshold:
                if self.mode == "track_signal_ESC":
                    await asyncio.sleep(10)
                    if self.signal.RSSI < self.threshold:
                        await self.set_mode("search")
                elif self.mode == "idle":
                    await self.set_mode("search")
            await asyncio.sleep(0.1)

    async def set_mode(self, mode):
        self.mode = mode
        if self.active_mode_task:
            self.active_mode_task.cancel()
            await asyncio.sleep(1)
        if mode == 'idle':
            self.active_mode_task = asyncio.create_task(self.idle())
        elif mode == 'track_signal_discrete':
            self.active_mode_task = asyncio.create_task(self.track_signal_discrete())
        elif mode == 'track_signal_SGD':
            self.active_mode_task = asyncio.create_task(self.track_signal_SGD())
        elif mode == 'track_signal_ESC':
            self.active_mode_task = asyncio.create_task(self.track_signal_ESC())
        elif mode == 'search':
            self.active_mode_task = asyncio.create_task(self.search())
        elif mode == 'track_trajectory':
            self.active_mode_task = asyncio.create_task(self.track_trajectory())
        else:
            print(f"No such mode '{mode}'!")
        logger.info(f"System mode set to - {mode}")

    async def idle(self):
        pass

    async def track_signal_discrete(self):  # 4 points
        await self.gimbal.set_speed(10, 10)
        await self.gimbal.set_acceleration(5, 5)
        algo = DiscreteTrackingAlgo(init_step_size=0.3, init_jitter_step=0.3)
        time_between_nodes = 0.4  # [sec]
        logger.info(f"time_between_nodes={time_between_nodes}")
        nominal_Az, nominal_El = self.gimbal.position
        signal_mean_prev = None
        while True:
            nodes_signals = []
            nominal_position = self.get_nominal_position()
            for node in algo.sample_nodes(nominal_Az+nominal_position[0], nominal_El+nominal_position[1]):  # go to nodes and save the signal at each node
                await self.gimbal.go_to(node[0], node[1])
                await asyncio.sleep(time_between_nodes)
                nodes_signals.append(self.signal.RSSI)
            signal_jitter = max(nodes_signals) - min(nodes_signals)
            signal_mean = sum(nodes_signals) / len(nodes_signals)
            if signal_jitter < 1:
                algo.increase_jitter()
            elif signal_jitter > 1:
                algo.decrease_jitter()
            if signal_mean_prev:
                signal_diff = abs(signal_mean-signal_mean_prev)
                algo.step_size = 0.2*signal_diff
                if algo.step_size < 0.05:
                    algo.step_size = 0.05
                elif algo.step_size > 0.3:
                    algo.step_size = 0.3
            signal_mean_prev = signal_mean
            nominal_Az, nominal_El = algo.next_position(nodes_signals, nominal_Az, nominal_El)

    async def track_signal_SGD(self):
        gradient_step = 0.07  # 0.2
        rssi_measure_delay = 0.05
        learning_rate = 0.08  # 0.5
        logger.info(f"Algo=SGD, gradient_step={gradient_step}, rssi_measure_delay={rssi_measure_delay}, lerning_rate={learning_rate}")

        await self.gimbal.set_speed(10, 10)
        await self.gimbal.set_acceleration(5, 5)

        async def calc_grad():
            position_0 = self.gimbal.position
            await asyncio.sleep(rssi_measure_delay)
            signal_0 = abs(self.signal.RSSI)
            await self.gimbal.go_to(position_0[0]+gradient_step, position_0[1], blocking=True)
            await asyncio.sleep(rssi_measure_delay)
            signal_dx = abs(self.signal.RSSI)
            await self.gimbal.go_to(position_0[0], position_0[1]+gradient_step, blocking=True)
            await asyncio.sleep(rssi_measure_delay)
            signal_dy = abs(self.signal.RSSI)
            grad_x = (signal_dx - signal_0) / gradient_step
            grad_y = (signal_dy - signal_0) / gradient_step
            return torch.tensor(grad_x), torch.tensor(grad_y)

        async def J(x, y):
            nominal_position = self.get_nominal_position()
            await self.gimbal.go_to(float(x)+nominal_position[0], float(y)+nominal_position[1], blocking=True)
            await asyncio.sleep(rssi_measure_delay)
            return torch.tensor(float(abs(self.signal.RSSI)), requires_grad=True)

        x = torch.tensor(self.gimbal.position[0], requires_grad=True)
        y = torch.tensor(self.gimbal.position[1], requires_grad=True)
        optimizer = optim.Adam([x, y], lr=learning_rate)
        while True:
            loss = await J(x, y)
            try:
                optimizer.zero_grad()
                x.grad, y.grad = await calc_grad()
                optimizer.step()
            except Exception as e:
                print(e)

    async def track_signal_ESC(self):
        await self.gimbal.set_speed(10, 10)
        await self.gimbal.set_acceleration(5, 5)

        az_axis_ESC = ESC(sample_freq=10, A=0.1, omega_Hz=1, phase=0, K=1, axis='Azimuth')
        el_axis_ESC = ESC(sample_freq=10, A=0.1, omega_Hz=1, phase=np.pi/2, K=1, axis='Elevation')
        await asyncio.gather(az_axis_ESC.run(self.signal, self.gimbal, self), el_axis_ESC.run(self.signal, self.gimbal, self))

    async def search(self):
        await self.gimbal.set_speed(10, 10)
        await self.gimbal.set_acceleration(5, 5)

        await Search(range=3, step_deg=0.5).run(self)

    async def track_trajectory(self):
        await self.gimbal.set_speed(10, 10)
        await self.gimbal.set_acceleration(5, 5)
        while True:
            nominal_position = self.get_nominal_position()
            await self.gimbal.go_to(nominal_position[0], nominal_position[1])
            await asyncio.sleep(0.1)

    def get_nominal_position(self):
        seconds_from_start = (datetime.now() - self.start_time).total_seconds()
        az = np.interp(seconds_from_start, self.trajectory['Time'], self.trajectory['Azimuth']) + 2  # offset that simulates bad knowledge of the trajectory
        el = np.interp(seconds_from_start, self.trajectory['Time'], self.trajectory['Elevation']) + 1
        return self.positioner_to_gimbal_axis((az, el))

    def read_next_trajectory_position(self):
        seconds_from_start = int((datetime.now()-self.start_time).total_seconds())
        try:
            line = self.trajectory.iloc[seconds_from_start+2]  # the next 2nd point in trajectory
        except Exception as e:
            line = self.trajectory.iloc[-1]
        return line['Azimuth'], line['Elevation']

    async def positioner_follow_trajectory(self):
        self.start_time = datetime.now()
        while True:
            az, el = self.read_next_trajectory_position()
            az_speed = (az - self.positioner.position[0]) / 2
            el_speed = (el - self.positioner.position[1]) / 2
            await self.positioner.go_to_at_speed(az, el, az_speed, el_speed)
            await asyncio.sleep(1)

    async def go_to_start_position(self, positioner_pos: tuple, gimbal_pos: tuple):
        await self.positioner.set_speed(20, 20)
        await self.positioner.go_to(positioner_pos[0], positioner_pos[1])
        await self.gimbal.go_to(gimbal_pos[0], gimbal_pos[1])
        while self.positioner.position != positioner_pos or self.gimbal.position != gimbal_pos:
            await asyncio.sleep(0.5)
        await self.set_mode("idle")


class DiscreteTrackingAlgo:
    def __init__(self, init_step_size, init_jitter_step):
        self.step_size = init_step_size
        self.jitter_step = init_jitter_step
        logger.info(f"Algo=DiscreteTrackingAlgo, init_step_size={init_step_size}, init_jitter_step={init_jitter_step}")

    def sample_nodes(self, current_Az, current_El):
        """ calculate AZ, EL of each sample node
        returns: [(az, el) x number of nodes]"""

        nodes_raw = [(current_Az+self.jitter_step, current_El), (current_Az, current_El+self.jitter_step), (current_Az-self.jitter_step, current_El), (current_Az, current_El-self.jitter_step)]
        nodes = [(round(node[0], 1), round(node[1], 1)) for node in nodes_raw]
        return nodes

    def next_position(self, nodes_signals, current_Az, current_El):
        """ calculate next antenna position
        gets: nodes_signals = (signal at node1, signal at node2...)"""

        total_Az_signal = nodes_signals[0]-nodes_signals[2]
        total_El_signal = nodes_signals[1] - nodes_signals[3]
        d = math.sqrt(math.pow(total_Az_signal, 2) + math.pow(total_El_signal, 2))
        if d == 0:
            return current_Az, current_El
        az_delta = (total_Az_signal*self.step_size)/d
        az_delta = math.copysign(az_delta, total_Az_signal)
        el_delta = (total_El_signal*self.step_size)/d
        el_delta = math.copysign(el_delta, total_El_signal)
        az = round(current_Az + az_delta, 1)
        el = round(current_El + el_delta, 1)
        return az, el

    def decrease_step(self):
        self.step_size = round(self.step_size-0.05, 2) if self.step_size > 0.05 else 0.05

    def increase_step(self):
        self.step_size = round(self.step_size+0.05, 2) if self.step_size < 0.3 else 0.3

    def decrease_jitter(self):
        self.jitter_step = round(self.jitter_step-0.05, 2) if self.jitter_step > 0.05 else 0.05

    def increase_jitter(self):
        self.jitter_step = round(self.jitter_step+0.05, 2) if self.jitter_step < 0.5 else 0.5


class ESC:
    def __init__(self, sample_freq, A, omega_Hz, phase, K, axis):
        self.sample_freq = sample_freq
        self.dt = 1/self.sample_freq
        self.A = A  # Perturbation amplitude
        self.omega = omega_Hz * 2 * np.pi
        self.phase = phase
        self.K = K  # integration gain
        self.axis = axis
        logger.info(f"Algo=ESC, sample_freq={sample_freq}, A={A}, omega_Hz={omega_Hz}, phase={omega_Hz}, K={K}")

    async def run(self, signal, gimbal, system):
        y0 = signal.RSSI
        if self.axis == 'Azimuth':
            u = gimbal.position[0] - system.get_nominal_position()[0]  # initial position
        elif self.axis == 'Elevation':
            u = gimbal.position[1] - system.get_nominal_position()[1]  # initial position

        # High pass filter
        butterorder = 1
        butterfreq = 2  # in Hz for 'high'
        b, a = butter(butterorder, butterfreq * self.dt * 2, 'high')
        ys = np.zeros(butterorder + 1) + y0
        HPF = np.zeros(butterorder + 1)
        uhat = u
        t = 0

        while True:
            yval = signal.RSSI
            for k in range(butterorder):
                ys[k] = ys[k + 1]
                HPF[k] = HPF[k + 1]
            ys[butterorder] = yval

            HPFnew = 0
            for k in range(butterorder + 1):
                HPFnew = HPFnew + b[k] * ys[butterorder - k]
            for k in range(1, butterorder + 1):
                HPFnew = HPFnew - a[k] * HPF[butterorder + 1 - k]
            HPFnew /= a[0]
            HPF[butterorder] = HPFnew

            xi = HPFnew * np.sin(self.omega * t + self.phase)
            uhat += xi * self.K * self.dt
            u = uhat + self.A * np.sin(self.omega * t + self.phase)
            if self.axis == 'Azimuth':
                await gimbal.go_to_az(u + system.get_nominal_position()[0])
            elif self.axis == 'Elevation':
                await gimbal.go_to_el(u + system.get_nominal_position()[1])
            await asyncio.sleep(self.dt)
            t = t + self.dt


class Search:
    def __init__(self, range, step_deg):
        self.range = range
        self.step_deg = step_deg
        logger.info(f"Algo=Search, range={range}")

    async def run(self, system):
        while True:
            for r in np.arange(self.step_deg, self.range, self.step_deg):
                for theta in np.arange(0, 360, 10):
                    nominal_position = system.get_nominal_position()
                    x = round(r * np.cos(np.deg2rad(theta)), 2) + nominal_position[0]
                    y = round(r * np.sin(np.deg2rad(theta)), 2) + nominal_position[1]
                    await system.gimbal.go_to(x, y)
                    await asyncio.sleep(0.1)
            for r in np.arange(self.range, self.step_deg, -self.step_deg):
                for theta in np.arange(0, 360, 10):
                    nominal_position = system.get_nominal_position()
                    x = round(r * np.cos(np.deg2rad(theta)), 2) + nominal_position[0]
                    y = round(r * np.sin(np.deg2rad(theta)), 2) + nominal_position[1]
                    await system.gimbal.go_to(x, y)
                    await asyncio.sleep(0.1)
