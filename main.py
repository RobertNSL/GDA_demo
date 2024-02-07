"""
Controller loop
"""

import asyncio
import aioconsole
from datetime import datetime, timedelta
import logging
from devices import *

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%Y/%m/%d %H:%M:%S',
                    filename='GDA_demo.log',
                    filemode='w')
logger = logging.getLogger(__name__)

positioner = Positioner('192.168.200.211', 65432)
gimbal = Newmark('COM5', 19200)
signal = Network_Analizer("192.168.200.88", start_freq=28, measure_freq=29, stop_freq=30)
system = System(mode="idle", positioner=positioner, gimbal=gimbal, signal=signal)


async def logging_task(logger, positioner, gimbal, signal, freq_sec):
    while True:
        logger.info(f'Positioner: {positioner.position}, Gimbal: {gimbal.position}, Signal: {signal.RSSI}')
        await asyncio.sleep(freq_sec)


async def cli(*objects):
    while True:
        cmd = await aioconsole.ainput('>>')
        logger.info(f"User cmd: {cmd}")
        if cmd == 'exit':
            break
        try:
            if cmd[-1] == ")":
                await eval(cmd)
            else:
                print(eval(cmd))
        except Exception as e:
            print(f"Wrong input - {e}")


async def main():
    # Start user interface
    cli_task = asyncio.create_task(cli(positioner, signal, gimbal, system))


    #  Connect positioner, gimbal, Network
    await asyncio.gather(signal.connect(), positioner.connect(), gimbal.connect())

    # Start logger
    asyncio.create_task(logging_task(logger, positioner, gimbal, signal, freq_sec=1))

    #  Move stuff
    await positioner.set_speed(5, 5)
    await positioner.go_to(0, 0)
    await gimbal.go_to(0, 0)
    await asyncio.sleep(3)
    await system.set_mode("idle")
    # await system.set_mode("track_signal")
    asyncio.create_task(system.read_trajectory())

    # asyncio.create_task(system.mode_manager())
    asyncio.create_task(system.positioner_follow_trajectory())

    await cli_task
    #  Disconnect devices
    await positioner.disconnect()
    await gimbal.disconnect()
    await signal.disconnect()


asyncio.run(main())
