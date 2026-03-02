import sys
import os
import time
import math
import serial
import serial.tools.list_ports
from vpython import canvas, box, vector, rate, color, arrow, label

BAUD = 115200

# Visual smoothing in quaternion space (0..1)
# Higher = more responsive, lower = smoother
Q_ALPHA = 0.25

def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)
    print("Available ports:")
    for p in ports:
        print(f"  {p.device} - {p.description}")
    return ports

def auto_pick_usb_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    # Prefer real USB-UART, avoid Bluetooth
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "bluetooth" in desc:
            continue
        if any(k in desc for k in ["usb serial", "ftdi", "cp210", "silicon labs", "ch340", "wch", "uart"]):
            return p.device
        if "vid:pid=0403:6001" in hwid:  # FTDI common
            return p.device
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return ports[0].device

def quat_norm(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)

def quat_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]

def quat_lerp(a, b, t):
    # Linear interpolate then normalize (fast “good enough” smoothing)
    w = (1-t)*a[0] + t*b[0]
    x = (1-t)*a[1] + t*b[1]
    y = (1-t)*a[2] + t*b[2]
    z = (1-t)*a[3] + t*b[3]
    return quat_norm((w, x, y, z))

def quat_to_rotmat(q):
    qw, qx, qy, qz = quat_norm(q)

    # Rotation matrix
    R00 = 1 - 2*(qy*qy + qz*qz)
    R01 = 2*(qx*qy - qw*qz)
    R02 = 2*(qx*qz + qw*qy)

    R10 = 2*(qx*qy + qw*qz)
    R11 = 1 - 2*(qx*qx + qz*qz)
    R12 = 2*(qy*qz - qw*qx)

    R20 = 2*(qx*qz - qw*qy)
    R21 = 2*(qy*qz + qw*qx)
    R22 = 1 - 2*(qx*qx + qy*qy)

    return (R00,R01,R02, R10,R11,R12, R20,R21,R22)

def apply_rotmat(R, body, x_axis, y_axis, z_axis):
    ex = vector(R[0], R[3], R[6])
    ey = vector(R[1], R[4], R[7])
    ez = vector(R[2], R[5], R[8])

    body.axis = ex
    body.up   = ez

    x_axis.pos = vector(0,0,0); x_axis.axis = 1.2 * ex
    y_axis.pos = vector(0,0,0); y_axis.axis = 1.2 * ey
    z_axis.pos = vector(0,0,0); z_axis.axis = 1.2 * ez

# ---------- Start ----------
print("Script path:", os.path.abspath(__file__))
list_ports()

PORT = sys.argv[1] if len(sys.argv) > 1 else auto_pick_usb_port()
if not PORT:
    print("Could not select a serial port.")
    sys.exit(1)

print("Using port:", PORT)
ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(1.0)

# Optional: flush junk / boot text
ser.reset_input_buffer()

scene = canvas(title="BMI160 Quaternion Viewer (smooth + no axis flips)",
               width=900, height=600, background=vector(0.95, 0.95, 0.95))

body = box(size=vector(2, 0.25, 0.6), color=vector(0.2, 0.2, 0.2))
x_axis = arrow(color=color.blue,  shaftwidth=0.05)
y_axis = arrow(color=color.green, shaftwidth=0.05)
z_axis = arrow(color=color.red,   shaftwidth=0.05)

txt = label(pos=vector(0,1.2,0), text="", height=14, box=False, color=color.black)

print("\nExpecting serial lines like:")
print("  DMPSTYLE,qw,qx,qy,qz,yaw,pitch,roll")
print("or you can change the prefix in code if your firmware prints differently.\n")

# Start with identity quaternion
q_prev = (1.0, 0.0, 0.0, 0.0)
q_disp = (1.0, 0.0, 0.0, 0.0)

# Read newest line only (prevents lag if MCU outputs faster than Python)
def read_latest_line():
    latest = None
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            latest = line
    return latest

while True:
    rate(60)

    line = read_latest_line()
    if not line:
        continue

    if not line.startswith("DMPSTYLE,"):
        continue

    parts = line.split(",")
    if len(parts) < 8:
        continue

    try:
        qw = float(parts[1]); qx = float(parts[2]); qy = float(parts[3]); qz = float(parts[4])
        yaw = float(parts[5]); pitch = float(parts[6]); roll = float(parts[7])
    except ValueError:
        continue

    q_new = quat_norm((qw, qx, qy, qz))

    # ✅ Hemisphere / continuity fix: prevent sudden sign flips
    if quat_dot(q_prev, q_new) < 0.0:
        q_new = (-q_new[0], -q_new[1], -q_new[2], -q_new[3])

    q_prev = q_new

    # ✅ Smooth in quaternion space (no Euler wrap issues)
    q_disp = quat_lerp(q_disp, q_new, Q_ALPHA)

    R = quat_to_rotmat(q_disp)
    apply_rotmat(R, body, x_axis, y_axis, z_axis)

    txt.text = (
        f"Port: {PORT}\n"
        f"q=({q_disp[0]:.3f},{q_disp[1]:.3f},{q_disp[2]:.3f},{q_disp[3]:.3f})\n"
        f"Yaw {yaw:.1f}°  Pitch {pitch:.1f}°  Roll {roll:.1f}°"
    )