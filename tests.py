import time
from datetime import datetime
from pyquaternion import Quaternion
import numpy as np
from devices import Newmark
import asyncio
import pandas as pd
import vxi11


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
        # logger.info(f"Network is connected - {spectrum_name}")
        await self.config()
        # logger.info("Network configed")

    async def disconnect(self):
        if self.instr is None:
            return

        self.instr.close()
        self.instr = None
        # logger.info("Network is disconnected")

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
        self.instr.write('SENS:AVER:COUN 15')
        self.instr.write(':DISP:WIND1:TRAC1:Y:RLEV -50')
        self.instr.write(':CALC:MARK1:STAT ON;X ' + str(self.measure_freq) + ' GHz')

    async def read(self):
        if self.instr is None:
            return

        buff = self.instr.ask(":CALC:MARK1:y?")
        amp = buff.split(',')[0]
        self.RSSI = float("{:05.1f}".format(float(amp)))

    async def save_snp(self, id):
        if self.instr is None:
            return
        self.instr.write(f':MMEM:STOR "D:/SnP/{id}.s2p"')


def rectengular_pattern(az_range=(-3, 3), el_range=(-3, 3), az_step=0.1, el_step=0.1):
    az_direction = True
    for el in np.arange(el_range[0], el_range[1]+el_step, el_step):
        if az_direction:
            azimuths = np.arange(az_range[0], az_range[1]+az_step, az_step)
        else:
            azimuths = np.arange(az_range[1], az_range[0]-az_step, -az_step)
        for az in azimuths:
            yield round(az, 2), round(el, 2)
        az_direction = not az_direction


gimbal = Newmark('192.168.200.59')
signal = Network_Analizer("192.168.200.88", start_freq=29.2, measure_freq=30.2, stop_freq=31.2)
log = {'idx': [], 'Az': [], 'El': []}


async def main():
    await asyncio.gather(signal.connect(), gimbal.connect())
    await gimbal.set_speed(10, 10)
    await gimbal.set_acceleration(5, 5)

    pattern_gen = rectengular_pattern()
    for idx, point in enumerate(pattern_gen):
        print(idx, point)
        await gimbal.go_to(point[0], point[1], blocking=True)
        await asyncio.sleep(1)
        await signal.read()
        log['idx'].append(idx)
        log['Az'].append(gimbal.position[0])
        log['El'].append(gimbal.position[1])
        await signal.save_snp(idx)

    pd.DataFrame(log).to_csv('beam_pattern.csv', index=False)

    await gimbal.disconnect()
    await signal.disconnect()


a = time.time()
asyncio.run(main())
code_duration = time.time()-a
print(f'Duration: {code_duration} sec')

"""

import pandas as pd
import matplotlib.pyplot as plt

#  3D plot (Az, El, Gain)
df = pd.read_csv('beam_pattern_flexible_20.2GHz.csv')
max_pos = df.loc[df['Signal'].idxmax()]

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_trisurf(df['Az'], df['El'], df['Signal'], cmap='viridis', edgecolor='none')
plt.xlim(-3, 3)
plt.ylim(-3, 3)

ax.text(max_pos['Az'], max_pos['El'], max_pos['Signal'], f"MAX - Az: {max_pos['Az']}, El: {max_pos['El']}, Signal: {max_pos['Signal']}", color='black')
ax.set_xlabel('Azimuth')
ax.set_ylabel('Elevation')
ax.set_zlabel('Signal')
plt.show()



def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle(vector1, vector2):
    vector1_unit = unit_vector(vector1)
    vector2_unit = unit_vector(vector2)
    dot = np.dot(vector1_unit, vector2_unit)
    if abs(dot) > 1:
        dot = 1*np.sign(dot)
    theta = np.arccos(dot)
    return theta  # [rad]


def polar_to_cartesian(az, el):  # az, el in deg
    az = np.deg2rad(az)
    el = np.deg2rad(el) + np.pi / 2
    xyz = np.array([np.sin(el) * np.cos(az), np.sin(el) * np.sin(az), np.cos(el)])
    return xyz


def cartesian_to_polar(xyz):
    az = np.arctan2(xyz[1], xyz[0])
    el = np.arccos(xyz[2] / np.sqrt(np.sum(xyz ** 2))) - np.pi / 2
    azel = np.array([az, el])
    azel = np.rad2deg(azel)
    if 180 < azel[0] < 360:
        azel[0] = azel[0] - 360
    return azel  # in [deg]


def get_quaternion_from_vectors(v1, v2):
    axis_rotation = unit_vector(np.cross(unit_vector(v1), unit_vector(v2)))
    angle_rotation = angle(v1, v2)
    return Quaternion(axis=axis_rotation, radians=angle_rotation)


def positioner_to_gimbal_axis(pos: tuple):
    theta = np.deg2rad(pos[0])
    phi = np.deg2rad(pos[1])
    az_gimbal = np.rad2deg(np.arctan(np.tan(phi) / np.cos(theta)))
    el_gimbal = np.rad2deg(np.arccos(((np.cos(phi))**2) * ((np.cos(theta))**2) + (np.sin(phi))**2))
    return az_gimbal, el_gimbal


print(positioner_to_gimbal_axis((30, 15)))

# gimbal_init_pointing = np.array([1, 0, 0])
# # gimbal_positioner_init = Quaternion(axis=[1, 0, 0], degrees=0)  # rotation to get from positioner axis to gimbal axis
# positioner_az = 45
# positioner_el = 10
# gimbal_az = 0
# gimbal_el = 0
#
# Qbc = Quaternion(axis=[0, 0, -1], degrees=positioner_az) * Quaternion(axis=[0, 1, 0], degrees=positioner_el)
# print(Qbc)
# v = Qbc.rotate(gimbal_init_pointing)
# Q_new = get_quaternion_from_vectors(v, gimbal_init_pointing)
# print(Q_new)
# v_new = Q_new.rotate(v)
# print(cartesian_to_polar(Q_new.rotate(np.array([1, 0, 0]))))
"""