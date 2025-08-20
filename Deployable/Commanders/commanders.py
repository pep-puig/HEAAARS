import time
from dronekit import VehicleMode
import RPi.GPIO as GPIO

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

import RPi.GPIO as GPIO
import time

class Servo:
    """
    Simple RC servo commander for hydrophone and water sensor spools
    using Raspberry Pi GPIO PWM.
    """

    def __init__(self, servo_type, gpio_pin, min_pwm=2.5, max_pwm=12.5):
        """
        :param servo_type: A string to identify which servo ('hydrophone' or 'water_sensor')
        :param gpio_pin: GPIO pin number the servo signal wire is connected to
        :param min_pwm: Duty cycle for wind direction
        :param max_pwm: Duty cycle for unwind direction
        """
        self.servo_type = servo_type
        self.gpio_pin = gpio_pin
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)  # Use BCM numbering
        GPIO.setup(self.gpio_pin, GPIO.OUT)

        # Initialize PWM at 50 Hz (typical for servos)
        self.pwm = GPIO.PWM(self.gpio_pin, 50)
        self.pwm.start(0)  # Initial duty cycle = 0 (no movement)

    def command_wind(self):
        """Rotate servo in 'wind' direction (minimum PWM)."""
        print(f"{self.servo_type} winding")
        self.pwm.ChangeDutyCycle(self.min_pwm)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  # Stop signal to prevent jitter

    def command_unwind(self):
        """Rotate servo in 'unwind' direction (maximum PWM)."""
        print(f"{self.servo_type} unwinding")
        self.pwm.ChangeDutyCycle(self.max_pwm)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  # Stop signal to prevent jitter

    def cleanup(self):
        """Release GPIO resources."""
        self.pwm.stop()
        GPIO.cleanup(self.gpio_pin)
