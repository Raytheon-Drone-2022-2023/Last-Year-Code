from mavsdk import System
import asyncio



async def run():
    drone = System()
    await drone.connect()

    await drone.action.arm()
    await drone.action.takeoff()

    await asyncio.sleep(5)

    await drone.action.goto_location(32.838058700000005, -96.78345776612298, 25, 0)

    #await drone.action.land()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())