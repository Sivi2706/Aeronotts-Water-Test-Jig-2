import sys
import time
import math
import serial
import serial.tools.list_ports
from vpython import *

BAUD = 115200

def auto_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "Bluetooth" not in p.description:
            return p.device
    return ports[0].device

PORT = sys.argv[1] if len(sys.argv) > 1 else auto_port()
ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(1)

scene = canvas(title="Orientation Viewer",
               width=900, height=600, background=color.white)

rocket = box(size=vector(2,0.3,0.6), color=color.gray(0.4))

def quat_to_rotmat(qw,qx,qy,qz):
    return (
        1-2*(qy*qy+qz*qz),
        2*(qx*qy-qw*qz),
        2*(qx*qz+qw*qy),
        2*(qx*qy+qw*qz),
        1-2*(qx*qx+qz*qz),
        2*(qy*qz-qw*qx),
        2*(qx*qz-qw*qy),
        2*(qy*qz+qw*qx),
        1-2*(qx*qx+qy*qy)
    )

while True:
    rate(100)
    line = ser.readline().decode(errors="ignore").strip()
    if not line.startswith("DMPSTYLE,"):
        continue

    parts = line.split(",")
    if len(parts) < 12:
        continue

    try:
        qw,qx,qy,qz = map(float, parts[1:5])
    except:
        continue

    R = quat_to_rotmat(qw,qx,qy,qz)

    ex = vector(R[0],R[3],R[6])
    ez = vector(R[2],R[5],R[8])

    rocket.axis = ex
    rocket.up = ez