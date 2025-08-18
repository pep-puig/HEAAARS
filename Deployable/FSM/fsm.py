from abc import ABC, abstractmethod
from enum import Enum, auto

# ----------------------
# StateEnum holds all state names
# ----------------------
class StateEnum(Enum):
    INIT = auto()
    TAKEOFF = auto()
    FLY_TO_TARGET = auto()
    LANDING_TARGET = auto()
    SWIM_TO_TARGET = auto()
    RETURN_HOME = auto()
    LANDING_HOME = auto()

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

class RobotSystem():
    def __init__(self):

        # Vehicle objects
        self.aerial = None
        self.aquatic = None

        # Handlers
        self.pose = PoseHandler()
        self.battery = BatteryHandler()
        self.parameters = ParameterHandler()      # Reads mission JSON
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

