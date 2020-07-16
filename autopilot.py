#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk import (OffboardError, PositionNedYaw)

from mapping import getWorld
from planning import dummyPath

async def run(path):
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Arming")
    while True:
        try:
            await drone.action.arm()
        except:
            print('Error arming, can\'t connect to the UAV !')
            await asyncio.sleep(1)
            continue
        else:
            break

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("Taking off...")
    for node in path:
        print('Going to {}'.format(node))
        await drone.offboard.set_position_ned(PositionNedYaw(node[1], node[0], -node[2], 0.0))
        await asyncio.sleep(5)

    print("Mission complete !")
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("Landing ...")
    await drone.action.land()


def start(planning):
    path = planning(None)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(path))
