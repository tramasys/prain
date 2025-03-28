import time
from sensors.lidar import LidarSensor

def lidar_test(bus: int, address: int = 0x10) -> None:
    print(f"Testing LiDAR on I2C bus {bus} with address {hex(address)}")
    # Create the sensor instance (polling ~20Hz)
    lidar = LidarSensor(bus=bus, address=address, poll_interval=0.05)
    lidar.start()

    try:
        for i in range(10):
            dist, flux, temp = lidar.get_data()

            if dist is None:
                print("No LiDAR data yet...")
            else:
                print(f"Distance: {dist} cm, Flux: {flux}, Temp (raw): {temp}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        print("LiDAR test finished")
