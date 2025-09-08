import time
from Deployable.Commanders.commanders import Servo

def main():
    # Create servo objects
    hydrophone_servo = Servo("hydrophone", gpio_pin=13, min_pwm=3.0, max_pwm=12.0)
    water_sensor_servo = Servo("water_sensor", gpio_pin=18, min_pwm=3.0, max_pwm=12.0)

    try:
        for _ in range(2):  # Wind/unwind each servo 3 times
            print("\nCycle start")

            # Hydrophone servo
            hydrophone_servo.command_wind()
            time.sleep(3)
            hydrophone_servo.command_unwind()
            time.sleep(3)

            # Water sensor servo
            water_sensor_servo.command_wind()
            time.sleep(3)
            water_sensor_servo.command_unwind()
            time.sleep(3)

            print("Cycle complete\n")

    finally:
        # Always cleanup to release GPIO
        hydrophone_servo.cleanup()
        water_sensor_servo.cleanup()

if __name__ == "__main__":
    main()
