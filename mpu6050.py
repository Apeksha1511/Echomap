# mpu6050.py — optional gyro heading and accelerometer step detection
import time, math
from config import MPU6050_ADDRESS, MPU6050_PWR
_bus=None

def setup_mpu6050():
    global _bus
    try:
        import smbus2
        _bus=smbus2.SMBus(1); _bus.write_byte_data(MPU6050_ADDRESS,MPU6050_PWR,0); time.sleep(.1)
        print('[MPU6050] Ready.'); return True
    except Exception as e: print(f'[MPU6050] Not found: {e}'); return False

def _word(reg):
    if _bus is None: return 0
    h=_bus.read_byte_data(MPU6050_ADDRESS,reg); l=_bus.read_byte_data(MPU6050_ADDRESS,reg+1); v=(h<<8)|l
    return v-65536 if v>=32768 else v

class HeadingTracker:
    def __init__(self): self.heading=0.; self.last_time=time.time(); self.available=False; self.last_accel=1.; self.last_step=0.
    def begin(self): self.available=setup_mpu6050(); self.last_time=time.time(); return self.available
    def update(self):
        if not self.available: return False
        now=time.time(); dt=min(now-self.last_time,.2); self.last_time=now
        gz=_word(0x47)/131.0
        if abs(gz)>2: self.heading=(self.heading+gz*dt)%360
        return self.step_detected()
    def step_detected(self):
        if not self.available: return False
        try:
            ax=_word(0x3B)/16384.; ay=_word(0x3D)/16384.; az=_word(0x3F)/16384.
            mag=math.sqrt(ax*ax+ay*ay+az*az); now=time.time(); changed=abs(mag-self.last_accel)>.18 and now-self.last_step>.35
            self.last_accel=mag
            if changed: self.last_step=now
            return changed
        except Exception: return False
    def set_heading(self,degrees): self.heading=float(degrees)%360
    def get_heading(self): return self.heading
tracker=HeadingTracker()
