import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

async def manual_altitude_takeoff(altitude):
    print(f"Taking off to {altitude}m")
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    
    while True:
        try:
            await drone.action.arm()
            break
        except Exception as e:
            print(f"Arming failed: {e}")
            await asyncio.sleep(1)
    
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, -0.5, 0.0)
    )
    await drone.offboard.start()
    
    target_reached = False
    async for position in drone.telemetry.position():
        current_altitude = abs(position.relative_altitude_m)
        print(f"Altitude: {current_altitude:.2f}m")
        
        if current_altitude >= altitude and not target_reached:
            print(f"Target altitude reached - hovering at {current_altitude}m")
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            target_reached = True
            break

asyncio.run(manual_altitude_takeoff(0.5))