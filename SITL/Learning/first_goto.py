from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

import argparse
parser = argparse.ArgumentParser(description='Commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 10

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(41.211700, 1.732736, 10)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(60)

print("Going towards first point for 30 seconds ...")
point3 = LocationGlobalRelative(41.215385, 1.732532, 0)
vehicle.simple_goto(point3)

print("Returned to Launch")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()