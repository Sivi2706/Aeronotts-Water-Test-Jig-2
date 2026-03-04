import sys
import time
import math
import serial
import serial.tools.list_ports
from vpython import *

BAUD = 115200

# -------- Apogee thresholds --------
ACC_ARM = 3.0      # m/s^2 above 1g
ACC_ZERO = 0.6     # near 0 linear accel
HOLD_SEC = 0.15

# -------- Port --------
def auto_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Bluetooth" not in p.description:
            return p.device
    return ports[0].device

PORT = sys.argv[1] if len(sys.argv) > 1 else auto_port()
ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(1)

# -------- Scene --------
scene = canvas(title="Apogee Viewer (Resultant Acceleration)",
               width=900, height=600, background=color.white)

rocket = box(size=vector(2,0.3,0.6), color=color.gray(0.4))

acc_arrow = arrow(color=color.orange, shaftwidth=0.1)
acc_arrow.pos = vector(0,0,0)

status = label(pos=vector(0,1.5,0), text="", height=18, box=False)

armed = False
apogee = False
below_start = None

while True:
    rate(100)
    line = ser.readline().decode(errors="ignore").strip()
    if not line.startswith("DMPSTYLE,"):
        continue

    parts = line.split(",")
    if len(parts) < 12:
        continue

    try:
        ax = float(parts[8])
        ay = float(parts[9])
        az = float(parts[10])
    except:
        continue

    a_mag = math.sqrt(ax*ax + ay*ay + az*az)

    # draw resultant vector
    if a_mag > 0:
        ux, uy, uz = ax/a_mag, ay/a_mag, az/a_mag
        acc_arrow.axis = vector(ux, uy, uz) * (0.2 * a_mag)

    # ----- Apogee detection -----
    lin_mag = abs(a_mag - 9.81)

    if not armed and lin_mag > ACC_ARM:
        armed = True

    if armed and not apogee:
        if lin_mag < ACC_ZERO:
            if below_start is None:
                below_start = time.time()
            elif time.time() - below_start > HOLD_SEC:
                apogee = True
        else:
            below_start = None

    status.text = f"|a| = {a_mag:.2f} m/s²\nArmed={armed}  Apogee={apogee}"