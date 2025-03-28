import threading
import time
from smbus2 import SMBus

class LidarSensor:
    """
    Basic driver for Benewake TF-Luna in I2C mode on a Raspberry Pi using bus /dev/i2c-1.
    Polls the sensor at ~20 Hz and parses 9-byte frames:
       Byte0 = 0x59
       Byte1 = 0x59
       Byte2 = Dist_L
       Byte3 = Dist_H
       Byte4 = Strength_L
       Byte5 = Strength_H
       Byte6 = Reserved or chip temperature
       Byte7 = Reserved or integration time
       Byte8 = Checksum
    """

    def __init__(self, bus_number=1, device_address=0x10):
        self.bus_number = bus_number
        self.device_address = device_address

        self._stop = False
        self._thread = None
        self._latest_reading = None

        self._bus = SMBus(self.bus_number)

    def start(self):
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop:
            try:
                # Read 9 bytes in a single transaction
                data = self._bus.read_i2c_block_data(self.device_address, 0, 9)
                # data is now a list of 9 integers [0..255]

                if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                    distance = data[2] + (data[3] << 8)
                    strength = data[4] + (data[5] << 8)
                    reserved_6 = data[6]
                    reserved_7 = data[7]
                    checksum = data[8]

                    calc_sum = sum(data[0:8]) & 0xFF
                    if calc_sum == checksum:
                        self._latest_reading = {
                            "distance_cm": distance,
                            "strength": strength,
                            "reserved_6": reserved_6,
                            "reserved_7": reserved_7,
                            "checksum": checksum,
                        }

            except OSError as e:
                print(f"I2C error from TF-Luna: {e}")

            except Exception as e:
                print(f"Unexpected error in TFLunaI2C: {e}")

            # Sleep ~50 ms => 20 Hz read
            time.sleep(0.05)

    def get_latest(self):
        """
        Return the most recent parsed dictionary or None if not available.
          {
            "distance_cm": 123,
            "strength": 456,
            "reserved_6": 0,
            "reserved_7": 0,
            "checksum": 217
          }
        """
        return self._latest_reading

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join()
        if self._bus:
            self._bus.close()
