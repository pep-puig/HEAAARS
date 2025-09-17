import time
from gpiozero import Servo
from time import sleep
from dronekit import VehicleMode
import RPi.GPIO as GPIO
from pymavlink import mavutil

class Navigator:
    """
    Navigation commander for aerial and aquatic vehicles using DroneKit.
    """

    def __init__(self):
        # Default speeds (can be overridden by params)
        self.aerial_climb_speed = 1.0
        self.aerial_flight_speed = 1.0
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

class ServoController:
    """
    Simple RC servo commander for hydrophone and water sensor spools
    using gpiozero Servo.
    """

    def __init__(self, servo_type, gpio_pin, min_value=-1, max_value=1):
        """
        :param servo_type: 'hydrophone' or 'water_sensor'
        :param gpio_pin: GPIO pin number the servo signal wire is connected to
        :param min_value: minimum servo value (-1 corresponds to full reverse)
        :param max_value: maximum servo value (+1 corresponds to full forward)
        """
        self.servo_type = servo_type
        self.gpio_pin = gpio_pin
        self.min_value = min_value
        self.max_value = max_value

        # Initialize gpiozero servo
        self.servo = Servo(self.gpio_pin, min_pulse_width=0.0006, max_pulse_width=0.0024)
        # 600-2400 µs range corresponds to min_pulse_width=0.0006, max_pulse_width=0.0024

    def command_wind(self):
        """Rotate servo in 'wind' direction (minimum)."""
        print(f"{self.servo_type} winding")
        self.servo.value = self.min_value
        sleep(20)
        self.servo.value = None  # Stop signal to prevent jitter

    def command_unwind(self):
        """Rotate servo in 'unwind' direction (maximum)."""
        print(f"{self.servo_type} unwinding")
        self.servo.value = self.max_value
        sleep(20)
        self.servo.value = None  # Stop signal to prevent jitter

    def cleanup(self):
        """Release resources (gpiozero handles this automatically)."""
        self.servo.close()
