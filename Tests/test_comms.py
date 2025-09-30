import sys
import os

# Add project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from Deployable.Commanders.commanders import Navigator

if __name__ == "__main__":
    nav = Navigator()

    success = nav.connect_pixhawk("/dev/ttyAMA3", 57600) # right_pixhawk -> serial1 / left_pixhawk ->ttyAMA3 

    if success:
        print("Pixhawk connection established, mode:", nav.vehicle.mode.name)
        nav.disconnect_pixhawk()
    else:
        print("Could not establish Pixhawk connection.")
