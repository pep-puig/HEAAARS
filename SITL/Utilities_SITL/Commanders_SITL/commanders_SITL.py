import time
from dronekit import VehicleMode
from pymavlink import mavutil

class Navigator:
    """
    Navigation commander for aerial and aquatic vehicles using DroneKit.
    """

    def __init__(self):
        # Default speeds (can be overridden by params)
        self.aerial_climb_speed = 1.0
        self.aerial_flight_speed = 3.0
        self.aquatic_swim_speed = 1.0

    # ------------------ VEHICLE STATUS CHECKS ------------------

    def is_guided_mode(self, vehicle):
        """Returns True if the vehicle is in GUIDED mode."""
        return vehicle.mode.name == "GUIDED"

    def is_armable(self, vehicle):
        """Returns True if the vehicle is armable."""
        return vehicle.is_armable

    def is_armed(self, vehicle):
        """Returns True if the vehicle is already armed."""
        return vehicle.armed

    # ------------------ PARAMETER SETTERS ------------------

    def set_aerial_navigation_params(self, climb_speed, flight_speed):
        """Set aerial movement parameters from mission config."""
        self.aerial_climb_speed = climb_speed
        self.aerial_flight_speed = flight_speed

    def set_aquatic_navigation_params(self, swim_speed):
        """Set aquatic movement parameters from mission config."""
        self.aquatic_swim_speed = swim_speed

    # ------------------ VEHICLE COMMANDS ------------------

    def arm_vehicle(self, vehicle):
        """
        Arms the vehicle without changing its mode.
        """
        vehicle.armed = True
        while not vehicle.armed:
            print("Arming vehicle...")
            time.sleep(0.5)
        print("Vehicle armed!")

    def disarm_aerial(self, vehicle, force=False):
        """
        Disarm the given vehicle safely using MAVLink.
        
        :param vehicle: DroneKit vehicle instance
        :param force: If True, force disarm even in air (use with caution!)
        """
        print("Disarming vehicle...")

        # Send MAVLink disarm command
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,            # confirmation
            0,            # param1 = 0 → disarm
            21196 if force else 0,  # param2 = 21196 → force disarm
            0, 0, 0, 0, 0
        )

        time.sleep(1)

        # Wait until the vehicle is disarmed
        while vehicle.armed:
            print(" Waiting for vehicle to disarm...")
            time.sleep(1)

        print("Vehicle disarmed successfully.")

    def disarm_aquatic(self, vehicle):
        """
        Switch Rover to HOLD mode and disarm.
        """
        print("Switching to HOLD mode...")
        vehicle.mode = VehicleMode("HOLD")

        time.sleep(1)

        # wait until mode changes
        while vehicle.mode.name != "HOLD":
            print(" Waiting for mode change...")
            time.sleep(0.5)

        print("Now disarming rover...")
        vehicle.armed = False

        time.sleep(1)

        # wait until disarmed
        while vehicle.armed:
            print(" Waiting for rover to disarm...")
            time.sleep(0.5)

        print("Rover disarmed successfully.")

    def take_off(self, vehicle, target_altitude):
        """
        Commands aerial vehicle to take off to target altitude.
        Assumes vehicle is already armed.
        """
        if not self.is_armed(vehicle):
            raise RuntimeError("Vehicle must be armed before takeoff")

        vehicle.mode = VehicleMode("GUIDED")
        vehicle.simple_takeoff(target_altitude)
        print(f"Taking off to {target_altitude} meters")

    def command_aerial_goto(self, vehicle, target_location):
        """
        Fly aerial vehicle to a given LocationGlobal or LocationGlobalRelative.
        """
        vehicle.simple_goto(target_location, groundspeed=self.aerial_flight_speed)

    def command_aquatic_goto(self, vehicle, target_location):
        """
        Command aquatic vehicle using simple_goto.
        """
        vehicle.simple_goto(target_location, groundspeed=self.aquatic_swim_speed)