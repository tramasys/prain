import time
from sensors.lidar import LidarSensor

def lidar_test(port: str, baudrate: int) -> None:
    print(f"Testing LiDAR on {port} at {baudrate}")
    lidar = LidarSensor(port=port, baudrate=baudrate)
    lidar.start()

    try:
        for i in range(10):
            scan = lidar.get_latest_scan()
            if scan is None:
                print("No LiDAR data yet...")
            else:
                print(f"Scan data: {scan}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        print("LiDAR test finished.")
