import time
from datetime import datetime

if __name__ == "__main__":
    try:
        while True:
            now = datetime.now()
            print(now.strftime("%Y-%m-%d %H:%M:%S"))
            time.sleep(5)   # print every 5 seconds
    except KeyboardInterrupt:
        print("Stopped by user")
