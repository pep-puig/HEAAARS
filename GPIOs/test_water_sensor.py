from Deployable.Commanders.commanders import WaterLandingHandler
import time

def main():
    water_sensor = WaterLandingHandler(pin=26)

    try:
        while True:
            if water_sensor.is_water():
                print("💧 Water detected!")
            else:
                print("No water detected.")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting test...")
    finally:
        water_sensor.cleanup()  # release GPIO

if __name__ == "__main__":
    main()
