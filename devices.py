import math
import gclib
import asyncio
import pandas as pd
import vxi11
from PyTic import *
from datetime import datetime, timedelta
import torch
import torch.optim as optim


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
        asyncio.create_task(self.read_position(0.5))

    async def disconnect(self):
        logger.info("Newmark disconnected")
        self.g.GClose()
        self.g = None

    async def go_to(self, az, el, blocking=False):
        self.g.GCommand(f'PAA={az*self.Az_steps_in_deg};PAB={el*self.El_steps_in_deg}')
        if blocking:
            self.g.GCommand('BG')
            self.g.GMotionComplete('AB')

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
        self.zero_position = (-58, 7)

    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(self.ip, self.port)
        asyncio.create_task(self.update_position(0.3))
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
        await self._send_cmd(self.az_controller._set_max_velocity(abs(self._az_to_steps(az_deg_sec)) * 800))
        await self._send_cmd(self.el_controller._set_max_velocity(abs(self._el_to_steps(el_deg_sec)) * 4000))

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
        az_pos = round(self._steps_to_az(rec[1]), 1)

        await self._send_cmd(self.el_controller.get_pos())
        rec = await self.reader.read(100)
        rec = eval(rec.decode())
        el_position_uncertain = rec[0]
        if el_position_uncertain:
            await self._send_cmd(self.el_controller.perform_homing(dir=0))
        el_pos = round(self._steps_to_el(rec[1]), 1)
        self.position = (az_pos-self.zero_position[0], el_pos-self.zero_position[1])

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
            await asyncio.sleep(0.1)
            await self.read()


class System:
    def __init__(self, mode, positioner, gimbal, signal):
        self.modes = ["idle", "track_signal_discrete", 'track_signal_SGD', 'track_signal_ESC', "search"]
        self.threshold = -65
        self.mode = mode
        self.active_mode_task = None
        self.positioner = positioner
        self.gimbal = gimbal
        self.signal = signal
        self.trajectory = pd.read_csv("positioner_trajectory.csv")
        self.next_trajectory_position = None

    def positioner_to_gimbal_axis(self, positioner_az, positioner_el):
        gimbal_az = -positioner_az
        gimbal_el = -positioner_el
        return gimbal_az, gimbal_el

    async def mode_manager(self):
        while True:
            if self.signal.RSSI > self.threshold and self.mode != "track_signal_discrete":
                await self.set_mode("track_signal_discrete")
            elif self.signal.RSSI < self.threshold and self.mode != "search":
                await self.set_mode("search")
            await asyncio.sleep(5)

    async def set_mode(self, mode):
        self.mode = mode
        if self.active_mode_task:
            self.active_mode_task.cancel()
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
            for node in algo.sample_nodes(nominal_Az, nominal_El):  # go to nodes and save the signal at each node
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
            trajectory_Az, trajectory_El = self.positioner_to_gimbal_axis(self.next_trajectory_position[0], self.next_trajectory_position[1])  # TODO
            nominal_Az, nominal_El = algo.next_position(nodes_signals, nominal_Az, nominal_El)

    async def track_signal_SGD(self):
        x = torch.tensor(self.gimbal.position[0], requires_grad=True)
        y = torch.tensor(self.gimbal.position[1], requires_grad=True)
        optimizer = optim.Adam([x, y], lr=0.3)
        while True:
            self.gimbal.go_to(x, y, blocking=True)
            await asyncio.sleep(0.1)
            func = abs(self.signal.RSSI)

            optimizer.zero_grad()
            func.backward()
            optimizer.step()

    async def track_signal_ESC(self):
        pass

    async def search(self):
        await self.gimbal.set_speed(10, 10)
        while True:
            nominal_az, nominal_el = -self.next_trajectory_position[0], self.next_trajectory_position[1]  # TODO
            await self.gimbal.go_to(nominal_az, nominal_el)
            await asyncio.sleep(1)

    async def read_trajectory(self):
        start_time = datetime.now()
        while True:
            seconds_from_start = int((datetime.now()-start_time).total_seconds())
            try:
                line = self.trajectory.iloc[seconds_from_start+1]
            except Exception as e:
                break
            self.next_trajectory_position = (line['Azimuth'], line['Elevation'])
            await asyncio.sleep(1)

    async def positioner_follow_trajectory(self):
        while True:
            az = self.next_trajectory_position[0]
            el = self.next_trajectory_position[1]
            az_speed = az - self.positioner.position[0]
            el_speed = el - self.positioner.position[1]
            await self.positioner.go_to_at_speed(az, el, az_speed, el_speed)
            await asyncio.sleep(1)


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
    def __init__(self):
        pass
    """
    freq = 100; % sample frequency
    dt = 1/freq;
    T = 10; % total period of simulation (in seconds)
    
    % perturbation parameters
    A = .2;  % amplitude
    omega = 10*2*pi; % 10 Hz
    phase = 0;
    K = 5;   % integration gain
    
    % high pass filter
    butterorder=1;
    butterfreq=2;  % in Hz for 'high'
    [b,a] = butter(butterorder,butterfreq*dt*2,'high')
    ys = zeros(1,butterorder+1)+y0;
    HPF=zeros(1,butterorder+1);
    
    uhat=u;
    for i=1:T/dt
        t = (i-1)*dt;
        time(i) = t;
        yvals(i)=J(u,t);
        
        for k=1:butterorder
            ys(k) = ys(k+1);
            HPF(k) = HPF(k+1);
        end
        ys(butterorder+1) = yvals(i);
        
        HPFnew = 0;
        for k=1:butterorder+1
            HPFnew = HPFnew + b(k)*ys(butterorder+2-k);
        end
        for k=2:butterorder+1
            HPFnew = HPFnew - a(k)*HPF(butterorder+2-k);
        end
        HPFnew = HPFnew/a(1);
        HPF(butterorder+1) = HPFnew;
        
        xi = HPFnew*sin(omega*t + phase);
        uhat = uhat + xi*K*dt;
        u = uhat + A*sin(omega*t + phase);
        uhats(i) = uhat;
        uvals(i) = u;    
    end
    """