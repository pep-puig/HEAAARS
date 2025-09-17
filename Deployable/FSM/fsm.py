from abc import ABC, abstractmethod
from enum import Enum, auto
from Commanders.commanders import Navigator, Servo
from Handlers.handlers import (
    PoseHandler,
    BatteryHandler,
    ParameterHandler,
    TimeHandler,
    WaterLandingHandler
)

# ----------------------
# StateEnum holds all state names
# ----------------------
class StateEnum(Enum):
    INIT = auto()
    TAKEOFF = auto()
    FLY2TARGET = auto()
    LANDINGATTARGET = auto()
    SWIM2TARGET = auto()
    RETURN2HOME = auto()
    LANDINGATHOME = auto()

# ----------------------
# State abstact class is a template for what all state will have
# ----------------------
class State(ABC):

    def __init__(self, state_id:StateEnum):
        self.state_id = state_id
        self.next_state = None

    @abstractmethod
    def run(self, robot):
        pass

# ----------------------
# FSM class is the state manager as it loops until there are no more states to go to
# ----------------------
class FSM:

    def __init__(self, robot, initial_state, states_dict):

        self.robot = robot
        self.states = states_dict
        self.current_state = self.states[initial_state]

    def run(self):
        """
        Runs the FSM until no next state is set.
        """
        while self.current_state is not None:
            print(f"Running state: {self.current_state.state_id}")
            
            # Execute current state logic
            self.current_state.run(self.robot)
            
            # Record previous state
            self.robot.previous_state = self.current_state.state_id
            
            # Determine next state
            next_state_enum = self.current_state.next_state
            if next_state_enum is None:
                print("FSM finished: no next state.")
                self.current_state = None
            else:
                self.current_state = self.states[next_state_enum]

# ----------------------
# RobotSystem holds vehicle type, handlers and commanders - all shared data is placed here
# ----------------------
class RobotSystem():
    def __init__(self):

        # Vehicle objects
        self.aerial = None
        self.aquatic = None

        # Handlers
        self.pose = PoseHandler()
        self.battery = BatteryHandler()
        self.parameters = ParameterHandler("params.json")      # Reads mission JSON
        self.time = TimeHandler()
        self.water_landing = WaterLandingHandler()

        # Commanders
        self.navigator = Navigator()              # Will be fed aerial or aquatic
        self.servo_hydrophone = Servo("hydrophone")
        self.servo_water_sensor = Servo("water_sensor")

        # Mission data
        self.params = {}           # Loaded JSON parameters
        self.home_location = None  # Recorded at takeoff
        self.previous_state = None # Keep track of previous state

