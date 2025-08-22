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
        if robot.aquatic is None:
            parser = argparse.ArgumentParser(description='Commands')
            parser.add_argument('--connect')
            args = parser.parse_args()
            connection_string = args.connect
            robot.aquatic = connect(connection_string, wait_ready=True)

        print(f"Battery level: {robot.aquatic.battery.level}%")

        # Check battery using BatteryHandler
        if robot.battery.is_empty(robot.aquatic, battery_threshold):
            print("Battery below threshold, aborting mission")
            self.next_state = None
            return

        # Record home location
        robot.home_location = robot.pose.get_home_location(robot.aquatic)
        print(f"Home location recorded: {robot.home_location}")

        # Wait for GUIDED mode
        while robot.aquatic.mode.name != "GUIDED":
            print("Waiting for aerial vehicle to enter GUIDED mode...")
            time.sleep(1)

        robot.time.start_mission_time = time.time()
        print(f"Mission start time recorded: {robot.time.start_mission_time}")

        robot.previous_state = StateEnum.INIT
        print("Init complete, transitioning to TakeOff")

        # Set the next state for FSM
        self.next_state = StateEnum.SWIM2TARGET
        #self.next_state = None

class Swim2Target(State):
    def __init__(self):
        super().__init__(StateEnum.SWIM2TARGET)

    def run(self, robot):
        print("Entering Swim2Target state...")
        robot.previous_state = StateEnum.SWIM2TARGET

        # --- Read params ---
        battery_threshold = robot.params["thresholds"]["min_battery_threshold"]
        target_lat = robot.params["target_parameters"]["swim_location"]["latitude"]
        target_lon = robot.params["target_parameters"]["swim_location"]["longitude"]
        swim_speed = robot.params["commanded_parameters"]["swim_speed"]
        distance_threshold = robot.params["thresholds"]["distance2location_threshold"]
        max_mission_time = robot.params["thresholds"]["max_mission_time"]

        # Pre-safety checks
        if  robot.battery.is_empty(robot.aquatic, battery_threshold):
            print("Battery too low before swim — switching to TakeOff")
            self.next_state = None
            return

        if target_lat is None or target_lon is None:
            print("Swim target coordinates missing — aborting mission / fallback")
            self.next_state = None
            return

        # Build target location (altitude 0 at surface level)
        target_location = LocationGlobalRelative(target_lat, target_lon, 0)

        # Command aquatic vehicle to swim
        robot.navigator.set_aquatic_navigation_params(swim_speed)
        robot.navigator.command_aquatic_goto(robot.aquatic, target_location)

        # Monitoring loop
        consecutive_not_decreasing = 0

        while True:
            # If reached
            if robot.pose.is_location_reached(robot.aquatic, target_location, distance_threshold):
                print("Swim target reached — returning to aerial TakeOff")
                self.next_state = StateEnum.MONITORING
                break

            # check decreasing
            if not robot.pose.is_distance2location_decreasing(robot.aquatic, target_location):
                consecutive_not_decreasing += 1
                print(f"Distance not decreasing for {consecutive_not_decreasing} sec")
            else:
                consecutive_not_decreasing = 0

            if consecutive_not_decreasing >= 10:
                print("Distance trend is flat for too long — switch to TakeOff")
                self.next_state = None
                break

            # Mission timeout check
            if max_mission_time is not None and \
               (time.time() - robot.time.start_mission_time) > max_mission_time:
                print("Mission timeout exceeded — fallback to TakeOff")
                self.next_state = None
                break

            # Battery mid-swim
            if robot.battery.is_empty(robot.aquatic, battery_threshold):
                print("Battery low during swim — aborting to TakeOff")
                self.next_state = None
                break

            time.sleep(1)

        # Disarm aquatic
        #print("Disarming aquatic vehicle after swim")
        #robot.navigator.disarm_aquatic(robot.aquatic)

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
        self.next_state = StateEnum.RETURN2HOME

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
        robot.navigator.command_aquatic_goto(robot.aquatic, home_location)

        while True:
            # Check if we have arrived back home
            if robot.pose.is_location_reached(robot.aquatic, home_location, distance_threshold):
                print("Reached home location – transitioning to LandingAtHome")
                self.next_state = None
                break

            # Mid-flight battery check
            if robot.battery.is_empty(robot.aquatic, battery_threshold):
                print("Battery low while returning home – still landing at home")
                self.next_state = None
                break

            time.sleep(0.5)

        print("Return2Home complete")