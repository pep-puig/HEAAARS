import time
from Utilities_SITL.FSM_SITL.fsm_SITL import FSM, RobotSystem
from Utilities_SITL.FSM_SITL.fsm_SITL import StateEnum
# from Deployable.Missions.fly_swim_to_states import Init, TakeOff, Fly2TargetLocation, LandingAtTarget, Swim2Target, Return2Home, LandingAtHome
from Missions_SITL.fly_land_monitor import Init, TakeOff, Fly2TargetLocation, LandingAtTarget, Monitoring, Return2Home, LandingAtHome

def main():
    # Create RobotSystem instance
    robot = RobotSystem()

    # Create concrete state instances
    init_state = Init()
    takeoff_state = TakeOff()
    fly2target_state = Fly2TargetLocation()
    landing_target_state = LandingAtTarget()
    monitoring_state = Monitoring()
    return_home_state = Return2Home()
    landing_home_state = LandingAtHome()

    # Build a dictionary of states for the FSM
    states_dict = {
        StateEnum.INIT: init_state,
        StateEnum.TAKEOFF: takeoff_state,
        StateEnum.FLY2TARGETLOCATION: fly2target_state,
        StateEnum.LANDINGATTARGET: landing_target_state,
        StateEnum.MONITORING: monitoring_state,
        StateEnum.RETURN2HOME: return_home_state,
        StateEnum.LANDINGATHOME: landing_home_state
    }

    # Initialize FSM with robot, initial state, and state dictionary
    fsm = FSM(robot, StateEnum.INIT, states_dict)

    # Run the FSM
    print("Starting mission FSM...")
    fsm.run()
    print("Mission complete.")

if __name__ == "__main__":
    main()
