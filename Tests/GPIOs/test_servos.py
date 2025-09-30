import time
from time import sleep
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
from Deployable.Commanders.commanders import ServoController

def main():
    # Initialize servos
    hydrophone_servo = ServoController("hydrophone", gpio_pin=13, min_value=-1, max_value=1)
    water_sensor_servo = ServoController("water_sensor", gpio_pin=18, min_value=-1, max_value=1)

    try:
        for cycle in range(2):  # wind/unwind each servo 2 times
            print(f"\n--- Cycle {cycle+1} start ---")

            # Hydrophone servo
            hydrophone_servo.command_wind()
            sleep(3)
            hydrophone_servo.command_unwind()
            sleep(3)

            # Water sensor servo
            water_sensor_servo.command_wind()
            sleep(3)
            water_sensor_servo.command_unwind()
            sleep(3)

            print(f"--- Cycle {cycle+1} complete ---\n")

    finally:
        # Cleanup to release GPIO
        hydrophone_servo.cleanup()
        water_sensor_servo.cleanup()
        print("GPIO cleaned up, test finished.")

if __name__ == "__main__":
    main()
