import time
from sensors.lidar import LidarSensor

def lidar_test(bus: int, address: int = 0x10) -> None:
    print(f"Testing LiDAR on I2C bus {bus} with address {hex(address)}")
    lidar = LidarSensor(bus=bus, address=address, poll_interval=0.05)
    lidar.start()

    try:
        while True:
            dist, flux, temp = lidar.get_data()

            if dist is None:
                print("no LiDAR data yet ...")
            else:
                print(f"distance: {dist} cm, flux: {flux}, temp (raw): {temp}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        print("LiDAR test finished")
