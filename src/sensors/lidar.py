import threading
import time
from smbus2 import SMBus

TFL_DIST_LO      = 0x00
TFL_DIST_HI      = 0x01
TFL_FLUX_LO      = 0x02
TFL_FLUX_HI      = 0x03
TFL_TEMP_LO      = 0x04
TFL_TEMP_HI      = 0x05

TFL_TICK_LO      = 0x06
TFL_TICK_HI      = 0x07
TFL_ERR_LO       = 0x08
TFL_ERR_HI       = 0x09
TFL_VER_REV      = 0x0A
TFL_VER_MIN      = 0x0B
TFL_VER_MAJ      = 0x0C

TFL_SAVE_SETTINGS= 0x20
TFL_SOFT_RESET   = 0x21
TFL_SET_I2C_ADDR = 0x22
TFL_SET_TRIG_MODE= 0x23
TFL_TRIGGER      = 0x24
TFL_DISABLE      = 0x25
TFL_FPS_LO       = 0x26
TFL_FPS_HI       = 0x27
TFL_SET_LO_PWR   = 0x28
TFL_HARD_RESET   = 0x29

FPS_1   = 0x01
FPS_2   = 0x02
FPS_5   = 0x05
FPS_10  = 0x0A
FPS_35  = 0x23
FPS_50  = 0x32
FPS_100 = 0x64

class LidarSensor:
    def __init__(self, bus: int = 1, address: int = 0x10, poll_interval: float = 0.05):
        """
        :param bus: I2C bus number on RPi (1 => /dev/i2c-1)
        :param address: 7-bit address of TF-Luna in I2C mode (often 0x10)
        :param poll_interval: time in seconds between background reads
        """
        self.bus_num = bus
        self.address = address
        self.poll_interval = poll_interval

        self._dist = None
        self._flux = None
        self._temp = None

        self._stop = False
        self._thread = None

        self._bus = SMBus(self.bus_num)

    def start(self):
        self._stop = False
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _poll_loop(self):
        while not self._stop:
            self._read_measurement()
            time.sleep(self.poll_interval)

    def _read_measurement(self):
        """
        Read 6 bytes from TFL_DIST_LO..TFL_TEMP_HI and store them in _dist, _flux, _temp.
        Each read_i2c_block_data operation does a repeated start, reading a contiguous
        set of registers.
        """
        try:
            data = self._bus.read_i2c_block_data(self.address, TFL_DIST_LO, 6)
            # data[0..5] => dist_lo, dist_hi, flux_lo, flux_hi, temp_lo, temp_hi
            dist = data[0] | (data[1] << 8)
            flux = data[2] | (data[3] << 8)
            temp = data[4] | (data[5] << 8)

            self._dist = dist
            self._flux = flux
            self._temp = temp
        except OSError as e:
            # E.g. if sensor is disconnected
            print(f"Error reading TF-Luna I2C: {e}")

    def get_data(self):
        return (self._dist, self._flux, self._temp)

    def stop(self):
        self._stop = True
        if self._thread is not None:
            self._thread.join()
        self._bus.close()

    def read_reg(self, reg: int) -> int:
        """
        Read one byte from register 'reg' using repeated start.
        Return the read byte as int.
        """
        # You can do this with a 1-byte read:
        data = self._bus.read_i2c_block_data(self.address, reg, 1)
        return data[0]

    def write_reg(self, reg: int, value: int) -> bool:
        """
        Write one byte (value) to register 'reg'.
        Return True on success.
        """
        try:
            self._bus.write_i2c_block_data(self.address, reg, [value])
            return True
        except OSError as e:
            print(f"I2C write error: {e}")
            return False

    def get_firmware_version(self) -> tuple[int,int,int] | None:
        """
        Read TFL_VER_REV, TFL_VER_MIN, TFL_VER_MAJ => (rev, minor, major).
        Return None on failure.
        """
        try:
            rev = self.read_reg(TFL_VER_REV)
            mn  = self.read_reg(TFL_VER_MIN)
            mj  = self.read_reg(TFL_VER_MAJ)
            return (rev, mn, mj)
        except OSError:
            return None

    def get_frame_rate(self) -> int | None:
        """
        Return the current frame rate as a 16-bit integer (fps).
        Or None on error.
        """
        try:
            lo = self.read_reg(TFL_FPS_LO)
            hi = self.read_reg(TFL_FPS_HI)
            return (hi << 8) | lo
        except OSError:
            return None

    def set_frame_rate(self, fps: int) -> bool:
        """
        Set frame rate (fps) as 16-bit integer.
        e.g., set_frame_rate(100) => 100 fps
        """
        lo = fps & 0xFF
        hi = (fps >> 8) & 0xFF
        ok_lo = self.write_reg(TFL_FPS_LO, lo)
        ok_hi = self.write_reg(TFL_FPS_HI, hi)
        return ok_lo and ok_hi

    def save_settings(self) -> bool:
        """Write 0x01 to TFL_SAVE_SETTINGS."""
        return self.write_reg(TFL_SAVE_SETTINGS, 0x01)

    def soft_reset(self) -> bool:
        """Write 0x02 to TFL_SOFT_RESET => reset and reboot."""
        return self.write_reg(TFL_SOFT_RESET, 0x02)

    def hard_reset(self) -> bool:
        """Write 0x01 to TFL_HARD_RESET => restore factory defaults."""
        return self.write_reg(TFL_HARD_RESET, 0x01)

    def set_i2c_addr(self, new_addr: int) -> bool:
        """
        Write new_addr to TFL_SET_I2C_ADDR. Must reboot for it to take effect.
        new_addr range is typically 0x08..0x77
        """
        return self.write_reg(TFL_SET_I2C_ADDR, new_addr)

    def set_enable(self) -> bool:
        """Write 0x01 to TFL_DISABLE => enable measurements."""
        return self.write_reg(TFL_DISABLE, 0x01)

    def set_disable(self) -> bool:
        """Write 0x00 to TFL_DISABLE => stop measurements."""
        return self.write_reg(TFL_DISABLE, 0x00)

    def set_continuous_mode(self) -> bool:
        """Write 0x00 to TFL_SET_TRIG_MODE => continuous sampling at frame rate."""
        return self.write_reg(TFL_SET_TRIG_MODE, 0x00)

    def set_trigger_mode(self) -> bool:
        """Write 0x01 to TFL_SET_TRIG_MODE => measurement only when triggered."""
        return self.write_reg(TFL_SET_TRIG_MODE, 0x01)

    def trigger_measurement(self) -> bool:
        """Write 0x01 to TFL_TRIGGER => trigger one measurement (if in trigger mode)."""
        return self.write_reg(TFL_TRIGGER, 0x01)
