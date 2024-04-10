import asyncio
from datetime import datetime, timedelta
import numpy as np
from PyTic import *
import subprocess
import re
import pandas as pd
import aioconsole


class Positioner:
    def __init__(self):
        self.az_controller = TicController(serialnumber='00314754', current_limit=2793, max_accel=1000000, max_deccel=1000000, max_speed=20000000, step_size=StepSizes.ONEQUARTER)
        self.el_controller = TicController(serialnumber='00305902', current_limit=2793, max_accel=1000000, max_deccel=1000000, max_speed=20000000, step_size=StepSizes.ONEQUARTER)
        self.position = None
        self.zero_position = (-59, 7.8)
        self.position_uncertain = False
        # asyncio.create_task(self.Tic_reset_timeout())

    async def Tic_reset_timeout(self):
        while True:
            await asyncio.sleep(0.5)
            await self._send_cmd(self.az_controller.reset_command_timeout())
            await self._send_cmd(self.el_controller.reset_command_timeout())

    async def _send_cmd(self, cmd: str):
        return subprocess.check_output(eval(cmd))

    def _az_to_steps(self, deg):
        AZ_Zero = 90
        azcal = 1 / (53.33 * 4)
        return int((deg - AZ_Zero) / azcal)

    def _steps_to_az(self, steps):
        AZ_Zero = 90
        azcal = 1 / (53.33 * 4)
        return steps * azcal + AZ_Zero

    def _el_to_steps(self, deg):
        MaxElevation = 18.8
        elcal = -13.9 / (1000 * 4)
        return int((deg - MaxElevation) / elcal)

    def _steps_to_el(self, steps):
        MaxElevation = 18.8
        elcal = -13.9 / (1000 * 4)
        return steps * elcal + MaxElevation

    async def set_speed(self, az_deg_sec, el_deg_sec):
        await self._send_cmd(self.az_controller._set_max_velocity(int(abs(az_deg_sec) * (53.33 * 4)) * 10000))
        await self._send_cmd(self.el_controller._set_max_velocity(int(abs(el_deg_sec) * (4000 / 13.9)) * 10000))

    async def go_to(self, az, el):
        await self._send_cmd(self.az_controller.move(self._az_to_steps(az + self.zero_position[0])))
        await self._send_cmd(self.el_controller.move(self._el_to_steps(el + self.zero_position[1])))

    async def go_to_at_speed(self, az, el, az_deg_sec, el_deg_sec):
        await self.set_speed(az_deg_sec, el_deg_sec)
        await self.go_to(az, el)

    async def get_position(self):
        def parse_status(status: str):
            match = re.search('Position uncertain:\s*', status)
            position_uncertain = True if status[match.end()] == 'Y' else False
            match = re.search(r'Current position:\s+-?\d+\n', status)
            match = re.search(r'-?\d+', match.group(0))
            current_position = int(match.group(0))
            return position_uncertain, current_position

        rec_az, rec_el = await asyncio.gather(self._send_cmd(self.az_controller.get_pos()), self._send_cmd(self.el_controller.get_pos()))
        position_uncertain_az, current_position = parse_status(rec_az.decode())
        az_pos = self._steps_to_az(current_position)
        # rec_el = await self._send_cmd(self.el_controller.get_pos())
        position_uncertain_el, current_position = parse_status(rec_el.decode())
        el_pos = self._steps_to_el(current_position)
        self.position = (round(az_pos - self.zero_position[0], 1), round(el_pos - self.zero_position[1], 1))
        if position_uncertain_az or position_uncertain_el:
            self.position_uncertain = True
        else:
            self.position_uncertain = False

    async def update_position(self, Hz):  # TODO: get_position() takes 0.2 sec so not possible to get high Hz
        while True:
            await asyncio.sleep(1/Hz)
            await self.get_position()


async def perform_homing(positioner):
    print('Homing...')
    await positioner._send_cmd(positioner.az_controller.energize())
    await positioner._send_cmd(positioner.el_controller.energize())
    await positioner.get_position()
    if positioner.position_uncertain:
        await positioner._send_cmd(positioner.az_controller.perform_homing())
        await positioner._send_cmd(positioner.el_controller.perform_homing(0))
    while positioner.position_uncertain:
        await asyncio.sleep(1)
        await positioner.get_position()
    print('Homing finished')


def read_trajectory(file='positioner_trajectory.csv'):
    df = pd.read_csv(file)
    az_traj = df['Azimuth'].to_numpy()
    el_traj = df['Elevation'].to_numpy()
    return az_traj, el_traj


# read trajectory from file
az_traj, el_traj = read_trajectory()
positioner = Positioner()


async def handle_client(reader, writer):
    print(f'Connected from: {writer.get_extra_info("peername")[0]}')
    request = None
    while request != 'exit':
        try:
            request = (await reader.read(255)).decode('utf8')
            print(f'Received cmd: {request}')
            if request[-1] == ")":
                asyncio.create_task(eval(request))
            else:
                print(eval(request))
            # writer.write(response.encode('utf8'))
            # await writer.drain()
        except Exception as e:
            print(e)
            break
    print(f'Disconnected from: {writer.get_extra_info("peername")[0]} ')
    writer.close()


async def cli(positioner):
    while True:
        cmd = await aioconsole.ainput('>>')
        if cmd == 'exit':
            break
        try:
            if cmd[-1] == ")":
                await eval(cmd)
            else:
                print(eval(cmd))
        except Exception as e:
            print(f"Wrong input - {e}")


async def run_server():
    server = await asyncio.start_server(handle_client, '192.168.200.211', 65432)
    async with server:
        await server.serve_forever()


async def track():
    print('started tracking')
    global az_traj, el_traj
    az_speed = np.diff(az_traj)
    el_speed = np.diff(el_traj)
    az_traj = az_traj[1:]
    el_traj = el_traj[1:]
    last_cmd_time = datetime.now()
    for az, el, az_s, el_s in zip(az_traj, el_traj, az_speed, el_speed):
        await positioner.go_to_at_speed(az, el, az_s, el_s)
        await asyncio.sleep(((last_cmd_time + timedelta(seconds=1)) - datetime.now()).total_seconds())
        last_cmd_time = datetime.now()


async def main():
    # perform homing if needed
    await perform_homing(positioner)

    asyncio.create_task(run_server())
    cli_task = asyncio.create_task(cli(positioner))


    # update positioner.position variable
    asyncio.create_task(positioner.update_position(Hz=2))

    # go to starting position
    await positioner.go_to_at_speed(az_traj[0], el_traj[0], 20, 20)
    while positioner.position != (az_traj[0], el_traj[0]):
        await asyncio.sleep(0.1)

    await cli_task

asyncio.run(main())
