import json
import os
import math
import time
from dronekit import LocationGlobal, LocationGlobalRelative

class PoseHandler:
    """
    Handles position-related computations for both location and altitude.
    """

    def __init__(self):
        # Used to compare decreasing errors
        self.prev_loc_error = None
        self.prev_alt_error = None

    def get_home_location(self, vehicle):
        """
        Called in InitState to record the vehicle's starting location.
        :param vehicle: DroneKit Vehicle instance
        :return: LocationGlobalRelative of current position
        """
        return self.get_current_location(vehicle)

    def get_current_location(self, vehicle):
        """
        Returns the current location of the vehicle.
        :param vehicle: DroneKit Vehicle instance
        :return: LocationGlobalRelative object
        """
        return vehicle.location.global_relative_frame

    def get_current_altitude(self, vehicle):
        """
        Returns the current altitude of the vehicle.
        :param vehicle: DroneKit Vehicle instance
        :return: Altitude in meters
        """
        return vehicle.location.global_relative_frame.alt

    def get_distance2location(self, vehicle, target_location):
        """
        Computes ground distance between current position and target_location.
        """
        current_loc = self.get_current_location(vehicle)
        return self.get_distance_metres(current_loc, target_location)

    def is_distance2location_decreasing(self, vehicle, target_location):
        """
        Checks if the distance to target_location is decreasing over time.
        """
        current_dist = self.get_distance2location(vehicle, target_location)
        decreasing = False
        if self.prev_loc_error is not None and current_dist < self.prev_loc_error:
            decreasing = True
        self.prev_loc_error = current_dist
        return decreasing

    def is_location_reached(self, vehicle, target_location, threshold=1.0):
        """
        Returns TRUE if within threshold meters of target.
        """
        dist = self.get_distance2location(vehicle, target_location)
        return dist <= threshold

    def get_distance2altitude(self, vehicle, target_altitude):
        """
        Returns absolute difference between current altitude and target altitude (in meters).
        """
        current_alt = self.get_current_altitude(vehicle)
        return abs(current_alt - target_altitude)

    def is_distance2altitude_decreasing(self, vehicle, target_altitude):
        """
        Checks if altitude error is decreasing.
        """
        current_err = self.get_distance2altitude(vehicle, target_altitude)
        decreasing = False
        if self.prev_alt_error is not None and current_err < self.prev_alt_error:
            decreasing = True
        self.prev_alt_error = current_err
        return decreasing

    def is_altitude_reached(self, vehicle, target_altitude, threshold=0.5):
        """
        Returns TRUE if altitude is within a threshold of target_altitude.
        """
        return self.get_distance2altitude(vehicle, target_altitude) <= threshold

    @staticmethod
    def get_distance_metres(aLocation1, aLocation2):
        """
        Direct ArduPilot helper for distance computation from lat/lon.
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
class BatteryHandler:
    """
    Handles battery level monitoring for both aerial and aquatic vehicles.
    Provides access to battery percentage and threshold-based check for low battery.
    """

    def __init__(self):
        """
        Initializes the BatteryHandler with a reference to the RobotSystem.
        This allows the handler to access any vehicle through robot (if needed).
        
        :param robot: RobotSystem instance containing vehicle objects.
        """
        pass
        # self.robot = robot

    def get_battery(self, vehicle):
        """
        Returns the current battery level for the specified vehicle (in volts or percentage depending on DroneKit).
        If the battery data is not available yet, returns 0 and prints a warning.
        
        :param vehicle: DroneKit Vehicle instance (robot.aerial or robot.aquatic).
        :return: Battery level value (float).
        """
        if vehicle is None:
            raise ValueError("Vehicle not connected")

        # DroneKit stores battery in vehicle.battery.level (typically a percentage or voltage).
        if vehicle.battery is None or vehicle.battery.level is None:
            print("Battery reading unavailable")
            return 0

        return vehicle.battery.level

    def is_empty(self, vehicle, threshold=22.8):
        """
        Checks if the battery of the specified vehicle is below a certain threshold.
        By default threshold is 22.8 volts (or can be a percentage threshold depending on battery spec).
        
        :param vehicle: DroneKit Vehicle instance whose battery to check.
        :param threshold: Battery level threshold below which battery is considered empty.
        :return: Boolean (True if below threshold, else False).
        """
        return self.get_battery(vehicle) < threshold
    
class TimeHandler:
    """
    Handles timing logic for mission timeout, water landing timeout,
    and correction timeout. Uses Python's time.time() for tracking.
    """

    def __init__(self):
        """
        :param robot: RobotSystem, used to access timing thresholds in robot.params
        """
        # self.robot = robot
        self.mission_start_time = None
        self.water_landing_start_time = None
        self.correction_start_time = None

    def start_counter(self, counter_name):
        """
        Starts one of the three timers: 'mission', 'water', or 'correction'.
        """
        current_time = time.time()

        if counter_name == "mission":
            self.mission_start_time = current_time
        elif counter_name == "water":
            self.water_landing_start_time = current_time
        elif counter_name == "correction":
            self.correction_start_time = current_time
        else:
            raise ValueError("Invalid counter name. Use 'mission', 'water', or 'correction'.")

    def is_mission_timeout(self):
        """
        Returns True if mission time exceeded threshold (from params).
        """
        if self.mission_start_time is None:
            return False
        elapsed = time.time() - self.mission_start_time
        threshold = self.robot.params.get("mission_timeout", 0)
        return elapsed >= threshold

    def is_water_landing_timeout(self):
        """
        Returns True if water-landing timer exceeded threshold (from params).
        """
        if self.water_landing_start_time is None:
            return False
        elapsed = time.time() - self.water_landing_start_time
        threshold = self.robot.params.get("water_landing_timeout", 0)
        return elapsed >= threshold

    def is_correction_timeout(self):
        """
        Returns True if correction timer exceeded threshold (from params).
        """
        if self.correction_start_time is None:
            return False
        elapsed = time.time() - self.correction_start_time
        threshold = self.robot.params.get("correction_timeout", 0)
        return elapsed >= threshold

class ParameterHandler:
    """
    Loads mission configuration parameters from a JSON file.
    Ensures that all required fields are present and returns a dictionary
    of mission parameters for use across the FSM.
    """

    def __init__(self, filepath="config.json"):
        """
        Initializes the ParameterHandler with a default JSON file path.
        
        :param filepath: Path to the JSON file containing mission parameters.
        """
        self.filepath = filepath

    def get_params(self):
        """
        Loads mission parameters from the JSON file, checks that required
        fields are present, and returns them as a dictionary.
        
        :raises FileNotFoundError: If the configuration file does not exist.
        :raises ValueError: If any required parameter is missing from the file.
        :return: Dictionary containing mission parameters.
        """

        # 1. Check that the file exists
        if not os.path.exists(self.filepath):
            raise FileNotFoundError(f"Parameter file not found: {self.filepath}")

        # 2. Load parameter dictionary from JSON
        with open(self.filepath, "r") as f:
            params = json.load(f)

        return params

