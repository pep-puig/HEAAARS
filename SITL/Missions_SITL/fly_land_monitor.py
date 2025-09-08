import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
from Utilities_SITL.FSM_SITL.fsm_SITL import StateEnum, State , FSM, RobotSystem
from Utilities_SITL.Handlers_SITL.handlers_SITL import PoseHandler, BatteryHandler, ParameterHandler, TimeHandler
from Utilities_SITL.Commanders_SITL.commanders_SITL import Navigator
import argparse

class Init(State):
    def __init__(self):
        super().__init__(StateEnum.INIT)

    def run(self, robot):
        print("Entering Init state...")

        # Load parameters
        robot.params = robot.parameters.get_params()
        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]

        # Connect vehicles if not connected
        if robot.aerial is None:
            parser = argparse.ArgumentParser(description='Commands')
            parser.add_argument('--connect')
            args = parser.parse_args()
            connection_string = args.connect
            robot.aerial = connect(connection_string, wait_ready=True)

        print(f"Battery level: {robot.aerial.battery.level}%")

        # Check battery using BatteryHandler
        if robot.battery.is_empty(robot.aerial, battery_threshold):
            print("Battery below threshold, aborting mission")
            self.next_state = None
            return

        # Record home location
        robot.home_location = robot.pose.get_home_location(robot.aerial)
        print(f"Home location recorded: {robot.home_location}")

        # Wait for GUIDED mode
        while robot.aerial.mode.name != "GUIDED":
            print("Waiting for aerial vehicle to enter GUIDED mode...")
            time.sleep(1)

        robot.time.start_mission_time = time.time()
        print(f"Mission start time recorded: {robot.time.start_mission_time}")

        robot.previous_state = StateEnum.INIT
        print("Init complete, transitioning to TakeOff")

        # Set the next state for FSM
        self.next_state = StateEnum.TAKEOFF
        #self.next_state = None


class TakeOff(State):
    def __init__(self):
        super().__init__(StateEnum.TAKEOFF)

    def run(self, robot):
        print("Entering TakeOff state...")  

        # Get params for this state    
        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]
        mission_altitude = robot.params["target_parameters"]["mission_altitude"]
        r2h_altitude = robot.params["target_parameters"]["r2h_altitude"]

        # Check battery levels before takeoff
        if robot.battery.is_empty(robot.aerial, battery_threshold):
            print("Battery below threshold before takeoff. Aborting mission.")
            self.next_state = None
            return

        # Determine target altitude and next state based on previous state
        if robot.previous_state == StateEnum.INIT:
            target_altitude = mission_altitude
            self.next_state = StateEnum.FLY2TARGET
        else:
            target_altitude = r2h_altitude
            self.next_state = StateEnum.RETURN2HOME

        # Arm the aerial vehicle
        print("Arming aerial vehicle...")
        while not robot.navigator.is_armable(robot.aerial):
            print("Waiting for aerial vehicle to become armable...")
            time.sleep(1)
        robot.navigator.arm_vehicle(robot.aerial)
        print("Aerial vehicle armed.")

        # figure out how to set vertical speed
        # climb_speed = robot.params.get("climb_speed", 2)  # default 2 m/s
        # robot.navigator.set_aerial_navigation_params(climb_speed=climb_speed)

        # Command takeoff
        print(f"Taking off to target altitude: {target_altitude} m")
        robot.navigator.take_off(robot.aerial, target_altitude)

        # Wait until target altitude is reached
        while not robot.pose.is_altitude_reached(robot.aerial, target_altitude):
            current_altitude = robot.pose.get_current_altitude(robot.aerial)
            print(f"Current altitude: {current_altitude:.1f} m, target: {target_altitude} m")
            time.sleep(0.5)

        robot.previous_state = StateEnum.TAKEOFF
        
        print(f"Target altitude reached: {target_altitude} m")

class Fly2TargetLocation(State):
    def __init__(self):
        super().__init__(StateEnum.FLY2TARGET)
    
    def run(self, robot):
        print("Entering Fly2TargetLocation state...")

        # Get params for this state
        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]
        target_lat = robot.params["target_parameters"]["fly_location"]["latitude"]
        target_lon = robot.params["target_parameters"]["fly_location"]["longitude"]
        mission_altitude = robot.params["target_parameters"]["mission_altitude"]
        distance_threshold = robot.params["thresholds"]["distance2location_threshold"]
        climb_speed = robot.params["commanded_parameters"]["climb_speed"]
        fly_speed = robot.params["commanded_parameters"]["fly_speed"]

        # Check battery before starting flight
        if robot.battery.is_empty(robot.aerial, battery_threshold):
            print("Battery below threshold, returning home")
            self.next_state = StateEnum.RETURN2HOME
            return

        # Get target location from parameters
        target_location = LocationGlobalRelative(target_lat, target_lon, mission_altitude)

        print(f"Commanding aerial vehicle to fly to target location: {target_location}")
        robot.navigator.set_aerial_navigation_params(climb_speed, fly_speed)
        robot.navigator.command_aerial_goto(robot.aerial, target_location)

        robot.previous_state = StateEnum.FLY2TARGETLOCATION

        # Loop until we reach the target or battery becomes low
        while not robot.pose.is_location_reached(robot.aerial, target_location, distance_threshold):
            time.sleep(0.5)

            # Check battery mid-flight
            if robot.battery.is_empty(robot.aerial, battery_threshold):
                print("Battery low during flight, returning home")
                self.next_state = StateEnum.RETURN2HOME
                break
        else:
            # Target reached
            print("Target location reached, transitioning to LandingAtTarget")
            self.next_state = StateEnum.LANDINGATTARGET

class LandingAtTarget(State):
    def __init__(self):
        super().__init__(StateEnum.LANDINGATTARGET)

    def run(self, robot):
        print("Entering LandingAtTarget state...")

        # Get params from json file
        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]
        target_lat = robot.params["target_parameters"]["fly_location"]["latitude"]
        target_lon = robot.params["target_parameters"]["fly_location"]["longitude"]
        water_landing_alt = robot.params["target_parameters"]["water_landing_altitude"]

        robot.previous_state = StateEnum.LANDINGATTARGET

        # Pre-descent battery check
        if robot.battery.is_empty(robot.aerial, battery_threshold):
            print("Battery below threshold, switching to Return2Home")
            self.next_state = StateEnum.RETURN2HOME
            return     

        if target_lat is None or target_lon is None or water_landing_alt is None:
            print("Landing target parameters missing, aborting mission")
            self.next_state = StateEnum.RETURN2HOME
            return

        # Create target LocationGlobalRelative
        target_location = LocationGlobalRelative(target_lat, target_lon, water_landing_alt)

        # Command descent
        print(f"Descending to target location at altitude {water_landing_alt} meters")
        robot.navigator.command_aerial_goto(robot.aerial, target_location)

        # Monitor descent using WaterLandingHandler
        start_time = time.time()
        max_landing_time = robot.params["thresholds"]["max_landing_time"]

        while not robot.pose.is_altitude_reached(robot.aerial, water_landing_alt):
            elapsed = time.time() - start_time
            if max_landing_time and elapsed > max_landing_time:
                print("Maximum landing time exceeded, switching to Return2Home")
                self.next_state = StateEnum.RETURN2HOME
                return

            current_altitude = robot.pose.get_current_altitude(robot.aerial)
            print(f"Current altitude: {current_altitude:.1f} m, target: {water_landing_alt} m")
            time.sleep(0.5)

        print("Water detected, landing complete")

        # Disarm aerial and arm aquatic using Navigator
        print("Disarming aerial and arming aquatic vehicle")
        robot.navigator.disarm_vehicle(robot.aerial, force=True)

        # Set next state
        self.next_state = StateEnum.MONITORING
        print("LandingAtTarget complete, transitioning to Swim2Target")

class Monitoring(State):
    def __init__(self):
        super().__init__(StateEnum.MONITORING)
    
    def run(self, robot):
        print("Entering Monitoring state...")

        # Load the timeout threshold from params
        max_monitor_time = robot.params["thresholds"]["max_monitoring_time"]
        if max_monitor_time is None:
            print("Monitoring timeout is not defined, defaulting to 0.")
            max_monitor_time = 0

        # Start local timer
        start_time = time.time()

        # Loop until time exceeded
        while (time.time() - start_time) < max_monitor_time:
            time.sleep(0.5)  # could be 1.0s if we want full-second resolution

        print("Monitoring period finished, returning to TakeOff.")
        # When monitoring time is reached, go back to TakeOff
        self.next_state = StateEnum.TAKEOFF

class Return2Home(State):
    def __init__(self):
        super().__init__(StateEnum.RETURN2HOME)

    def run(self, robot):
        print("Entering Return2Home state...")

        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]
        distance_threshold = robot.params["thresholds"]["distance2location_threshold"]

        # Make sure home_location was recorded in Init
        home_location = robot.home_location

        print(f"Flying back to home location: {home_location}")
        robot.navigator.command_aerial_goto(robot.aerial, home_location)

        while True:
            # Check if we have arrived back home
            if robot.pose.is_location_reached(robot.aerial, home_location, distance_threshold):
                print("Reached home location – transitioning to LandingAtHome")
                self.next_state = StateEnum.LANDINGATHOME
                break

            # Mid-flight battery check
            if robot.battery.is_empty(robot.aerial, battery_threshold):
                print("Battery low while returning home – still landing at home")
                self.next_state = StateEnum.LANDINGATHOME
                break

            time.sleep(0.5)

        print("Return2Home complete")

class LandingAtHome(State):
    def __init__(self):
        super().__init__(StateEnum.LANDINGATHOME)

    def run(self, robot):
        print("Entering LandingAtHome state...")

        # Read altitude landing threshold from parameters
        home_alt_threshold = robot.params["thresholds"].get("homelandingdistance_threshold", 0.2)

        # Switch aerial drone into LAND mode
        print("Setting aerial vehicle mode to LAND")
        robot.aerial.mode = VehicleMode("LAND")

        # Monitor descent until on ground
        while True:
            current_alt = robot.pose.get_current_altitude(robot.aerial)
            print(f"Current altitude: {current_alt:.2f} m")

            if current_alt <= home_alt_threshold:
                print("Altitude below threshold, considered landed.")
                break

            time.sleep(0.5)

        print("LandingAtHome complete. Mission finished.")
        self.next_state = None