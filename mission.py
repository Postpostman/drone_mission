from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 100

print("Connecting to vehicle...")
vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
print("Connected.")

print("Waiting for vehicle to be armable...")
while not vehicle.is_armable:
    time.sleep(1)

print("Changing mode to ALT_HOLD...")
vehicle.mode = VehicleMode("ALT_HOLD")
while not vehicle.mode.name == 'ALT_HOLD':
    time.sleep(0.5)

vehicle.armed = True
print("Arming motors...")
while not vehicle.armed:
    time.sleep(1)

print("Taking off...")
start = time.time()
takeoff_throttle = 2000
vehicle.channels.overrides["3"] = takeoff_throttle

hover_throttle = None

while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {alt:.1f} m")

    if alt >= TARGET_ALT:
        print("Target altitude reached.")
        if hover_throttle is None:
            hover_throttle = vehicle.channels.overrides["3"]
            print(f"Hover throttle set to {hover_throttle}")
        break

    if time.time() - start > 1 and alt < TARGET_ALT:
        print("Altitude too low, increasing throttle...")
        vehicle.channels.overrides["3"] = 2100

    if time.time() - start > 60:
        print("Failed to reach target altitude.")
        break

    time.sleep(1)

if hover_throttle is not None:
    vehicle.channels.overrides["3"] = hover_throttle


def get_bearing(lat1, lon1, lat2, lon2):
    delta_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


bearing_to_B = get_bearing(*POINT_A, *POINT_B)
print(f"Bearing to Point B: {bearing_to_B:.2f}°")

print("Moving forward to Point B...")

vehicle.channels.overrides["2"] = 1600
time.sleep(15)
vehicle.channels.overrides["2"] = 1500
print("Arrived at Point B.")


def condition_yaw(heading, relative=False):
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading, 0, 1, is_relative, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


print("Rotating to heading 350°...")
condition_yaw(350)
time.sleep(5)

vehicle.channels.overrides = {}
print("Mission complete.")

vehicle.close()
