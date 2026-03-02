import sys
import os
import time
import math
import serial
import serial.tools.list_ports
from vpython import canvas, box, vector, rate, color, arrow, label

BAUD = 115200

# Quaternion display smoothing (0..1)
Q_ALPHA = 0.25

# Rotate the model +90° yaw initially
MODEL_YAW_OFFSET_DEG = 90.0

# Acceleration arrow scaling (bigger = longer arrow)
ACC_SCALE = 0.18

G0 = 9.80665  # m/s^2

# -------------------------
# Apogee detection tuning (accel-based "coast/apogee-like" cue)
# -------------------------
# This is NOT as robust as barometer-based apogee, but works as a visual mode indicator.
# Condition: magnitude close to ~0g (near ballistic/free-fall) for a minimum time.
A_MAG_THRESHOLD = 2.0          # m/s^2  (|a| below this implies near free-fall)
APOGEE_HOLD_SEC = 0.35         # seconds condition must hold continuously
HYST_RESET_THRESHOLD = 4.0     # m/s^2  (to drop back to IN FLIGHT if |a| rises above this)

# -------------------------
# Port selection
# -------------------------
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
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "bluetooth" in desc:
            continue
        if any(k in desc for k in ["usb serial", "ftdi", "cp210", "silicon labs", "ch340", "wch", "uart"]):
            return p.device
        if "vid:pid=0403:6001" in hwid:
            return p.device
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return ports[0].device

# -------------------------
# Quaternion helpers
# -------------------------
def quat_norm(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)

def quat_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]

def quat_lerp(a, b, t):
    w = (1-t)*a[0] + t*b[0]
    x = (1-t)*a[1] + t*b[1]
    y = (1-t)*a[2] + t*b[2]
    z = (1-t)*a[3] + t*b[3]
    return quat_norm((w, x, y, z))

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw
    )

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def yaw_offset_quat(deg):
    half = math.radians(deg) * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))

Q_MODEL = yaw_offset_quat(MODEL_YAW_OFFSET_DEG)

def rotate_vec_by_quat(v, q):
    q = quat_norm(q)
    t = quat_mul(q, (0.0, v[0], v[1], v[2]))
    r = quat_mul(t, quat_conj(q))
    return (r[1], r[2], r[3])

def quat_to_rotmat(q):
    qw, qx, qy, qz = quat_norm(q)
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

# -------------------------
# Serial setup
# -------------------------
print("Script path:", os.path.abspath(__file__))
list_ports()

PORT = sys.argv[1] if len(sys.argv) > 1 else auto_pick_usb_port()
if not PORT:
    print("Could not select a serial port.")
    sys.exit(1)

print("Using port:", PORT)
ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(1.0)
ser.reset_input_buffer()

def read_latest_line():
    latest = None
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            latest = line
    return latest

# -------------------------
# VPython scene + UI boxes
# -------------------------
scene = canvas(
    title=f"BMI160 Viewer + Apogee Mode (yaw offset {MODEL_YAW_OFFSET_DEG:.0f}°)",
    width=1100, height=700,
    background=vector(0.95, 0.95, 0.95)
)

# 3D model and axes
body = box(size=vector(2, 0.25, 0.6), color=vector(0.2, 0.2, 0.2))
x_axis = arrow(color=color.blue,  shaftwidth=0.05)
y_axis = arrow(color=color.green, shaftwidth=0.05)
z_axis = arrow(color=color.red,   shaftwidth=0.05)

# Acceleration resultant arrow
acc_arrow = arrow(color=vector(1.0, 0.6, 0.0), shaftwidth=0.08)
acc_arrow.pos = vector(0, 0, 0)
acc_arrow.axis = vector(0, 0, 0)

# Status "highlight boxes" (2D-like overlay built from thin 3D boxes)
# Place them in front of camera using fixed coordinates.
flight_box = box(pos=vector(-3.2, 2.2, 0), size=vector(3.0, 0.9, 0.02),
                 color=color.red, opacity=0.25)
apogee_box = box(pos=vector(3.2, 2.2, 0), size=vector(3.0, 0.9, 0.02),
                 color=color.green, opacity=0.10)

flight_lbl = label(pos=flight_box.pos, text="IN FLIGHT", height=16, box=False, color=color.black)
apogee_lbl = label(pos=apogee_box.pos, text="APOGEE MODE", height=16, box=False, color=color.black)

txt = label(pos=vector(0, 1.5, 0), text="", height=14, box=False, color=color.black)

print("\nExpected serial format (for accel arrow + apogee logic):")
print("DMPSTYLE,qw,qx,qy,qz,yaw,pitch,roll,ax,ay,az,amag\n")

# -------------------------
# State for smoothing + apogee detection
# -------------------------
q_prev = (1.0, 0.0, 0.0, 0.0)
q_disp = (1.0, 0.0, 0.0, 0.0)

# Apogee detection timer
below_thresh_start = None
apogee_detected = False

# -------------------------
# Main loop
# -------------------------
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

    # Parse quaternion + ypr
    try:
        qw = float(parts[1]); qx = float(parts[2]); qy = float(parts[3]); qz = float(parts[4])
        yaw = float(parts[5]); pitch = float(parts[6]); roll = float(parts[7])
    except ValueError:
        continue

    q_new = quat_norm((qw, qx, qy, qz))

    # Prevent sudden quaternion sign flips
    if quat_dot(q_prev, q_new) < 0.0:
        q_new = (-q_new[0], -q_new[1], -q_new[2], -q_new[3])
    q_prev = q_new

    # Smooth quaternion for display
    q_disp = quat_lerp(q_disp, q_new, Q_ALPHA)

    # Apply model yaw offset
    q_render = quat_mul(Q_MODEL, q_disp)

    # Rotate model
    R = quat_to_rotmat(q_render)
    apply_rotmat(R, body, x_axis, y_axis, z_axis)

    # Parse accel if present (required for arrow + apogee logic)
    have_accel = (len(parts) >= 12)
    if not have_accel:
        acc_arrow.axis = vector(0, 0, 0)
        txt.text = (
            f"Port: {PORT}\n"
            f"Yaw {yaw:.1f}°  Pitch {pitch:.1f}°  Roll {roll:.1f}°\n"
            f"NO ACCEL FIELDS: firmware must print ax,ay,az,amag"
        )
        continue

    try:
        ax = float(parts[8]); ay = float(parts[9]); az = float(parts[10])
        amag = float(parts[11])
    except ValueError:
        continue

    # Rotate accel (body) -> world frame
    axw, ayw, azw = rotate_vec_by_quat((ax, ay, az), q_render)
    a_mag = math.sqrt(axw*axw + ayw*ayw + azw*azw)

    # Draw resultant accel arrow
    if a_mag < 1e-6:
        acc_arrow.axis = vector(0, 0, 0)
    else:
        ux, uy, uz = axw/a_mag, ayw/a_mag, azw/a_mag
        acc_arrow.pos = vector(0, 0, 0)
        acc_arrow.axis = vector(ux, uy, uz) * (ACC_SCALE * a_mag)

    # -------------------------
    # Apogee detection (simple accel-magnitude cue)
    # -------------------------
    nowt = time.time()

    if not apogee_detected:
        if a_mag < A_MAG_THRESHOLD:
            if below_thresh_start is None:
                below_thresh_start = nowt
            elif (nowt - below_thresh_start) >= APOGEE_HOLD_SEC:
                apogee_detected = True
        else:
            below_thresh_start = None
    else:
        # Optional reset (if accel rises again significantly)
        if a_mag > HYST_RESET_THRESHOLD:
            apogee_detected = False
            below_thresh_start = None

    # Update "highlight boxes"
    if apogee_detected:
        flight_box.opacity = 0.08
        apogee_box.opacity = 0.35
    else:
        flight_box.opacity = 0.35
        apogee_box.opacity = 0.08

    txt.text = (
        f"Port: {PORT}\n"
        f"Yaw {yaw:.1f}°  Pitch {pitch:.1f}°  Roll {roll:.1f}°\n"
        f"|a|={a_mag:.2f} m/s²   |a|-g={(a_mag - G0):+.2f} m/s²\n"
        f"Apogee={'YES' if apogee_detected else 'NO'}  "
        f"(th<{A_MAG_THRESHOLD:.1f} for {APOGEE_HOLD_SEC:.2f}s)"
    )