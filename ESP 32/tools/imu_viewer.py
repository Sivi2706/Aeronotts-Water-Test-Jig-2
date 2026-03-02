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
# Apogee detection (orientation-agnostic "up" from resultant acceleration direction)
# -------------------------
# Idea:
# - Define the rocket's "up axis" as the direction of the measured acceleration vector in world frame
#   during powered/ascent. This adapts automatically to orientation.
# - Remove gravity to estimate linear acceleration, project onto that "up axis", integrate to get v_up.
# - Detect apogee when v_up falls from positive to ~0 after being clearly positive.
#
# Notes:
# - Accel-only apogee is NOT as robust as barometer apogee.
# - For hand tests ("move up then stop"), this works much better than free-fall logic.
#
ACC_LP = 0.25          # accel low-pass (0..1), higher = more responsive
VEL_LEAK = 0.02        # 0..1 per update (leaky integrator to limit drift)
VEL_ARM = 0.30         # m/s: must exceed this upward speed to "arm"
VEL_APOGEE = 0.05      # m/s: apogee when v_up drops below this
APOGEE_HOLD = 0.15     # s: must stay below threshold continuously

# How quickly the "up axis" adapts to new direction (0..1)
UP_AXIS_ALPHA = 0.08   # small = stable, large = very reactive

# Hysteresis reset (optional) – if you want it to re-arm automatically
RESET_VEL = 0.50       # m/s: if v_up becomes strongly positive again, reset apogee_detected


# =========================
# Port selection
# =========================
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


# =========================
# Quaternion helpers
# =========================
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


# =========================
# Vector helpers
# =========================
def v_norm(x, y, z):
    n = math.sqrt(x*x + y*y + z*z)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 0.0)
    return (x/n, y/n, z/n, n)

def v_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def v_lerp_unit(u_old, u_new, alpha):
    # Blend then renormalize
    x = (1-alpha)*u_old[0] + alpha*u_new[0]
    y = (1-alpha)*u_old[1] + alpha*u_new[1]
    z = (1-alpha)*u_old[2] + alpha*u_new[2]
    unx, uny, unz, n = v_norm(x, y, z)
    if n < 1e-12:
        return u_old
    return (unx, uny, unz)


# =========================
# Serial setup
# =========================
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


# =========================
# VPython scene + status boxes
# =========================
scene = canvas(
    title=f"BMI160 Viewer + Apogee (Up from resultant accel) | yaw offset {MODEL_YAW_OFFSET_DEG:.0f}°",
    width=1200, height=740,
    background=vector(0.95, 0.95, 0.95)
)

# Model and axes
body = box(size=vector(2, 0.25, 0.6), color=vector(0.2, 0.2, 0.2))
x_axis = arrow(color=color.blue,  shaftwidth=0.05)
y_axis = arrow(color=color.green, shaftwidth=0.05)
z_axis = arrow(color=color.red,   shaftwidth=0.05)

# Accel resultant arrow (orange)
acc_arrow = arrow(color=vector(1.0, 0.6, 0.0), shaftwidth=0.08)
acc_arrow.pos = vector(0, 0, 0)
acc_arrow.axis = vector(0, 0, 0)

# "Up axis" arrow (purple) – estimated ascent direction
up_arrow = arrow(color=vector(0.6, 0.2, 0.8), shaftwidth=0.06)
up_arrow.pos = vector(0, 0, 0)
up_arrow.axis = vector(0, 0, 0)

# Status "highlight boxes"
flight_box = box(pos=vector(-3.4, 2.4, 0), size=vector(3.2, 0.95, 0.02),
                 color=color.red, opacity=0.35)
apogee_box = box(pos=vector(3.4, 2.4, 0), size=vector(3.2, 0.95, 0.02),
                 color=color.green, opacity=0.08)

flight_lbl = label(pos=flight_box.pos, text="IN FLIGHT", height=16, box=False, color=color.black)
apogee_lbl = label(pos=apogee_box.pos, text="APOGEE MODE", height=16, box=False, color=color.black)

txt = label(pos=vector(0, 1.65, 0), text="", height=14, box=False, color=color.black)

print("\nExpected serial format (required for apogee):")
print("DMPSTYLE,qw,qx,qy,qz,yaw,pitch,roll,ax,ay,az,amag\n")

# =========================
# State
# =========================
q_prev = (1.0, 0.0, 0.0, 0.0)
q_disp = (1.0, 0.0, 0.0, 0.0)

# Up axis estimate in WORLD frame (unit vector)
# Initialize to +Z
u_up = (0.0, 0.0, 1.0)

# Vertical (along u_up) linear accel and velocity
a_up_lp = 0.0
v_up = 0.0

armed = False
apogee_detected = False
below_start = None
last_t = time.time()

# =========================
# Main loop
# =========================
while True:
    rate(60)

    line = read_latest_line()
    if not line:
        continue
    if not line.startswith("DMPSTYLE,"):
        continue

    parts = line.split(",")
    if len(parts) < 12:
        # no accel fields -> can't do apogee
        continue

    # Parse quaternion + ypr + accel
    try:
        qw = float(parts[1]); qx = float(parts[2]); qy = float(parts[3]); qz = float(parts[4])
        yaw = float(parts[5]); pitch = float(parts[6]); roll = float(parts[7])
        ax = float(parts[8]); ay = float(parts[9]); az = float(parts[10])
        amag_body = float(parts[11])
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

    # Rotate accel into world frame
    axw, ayw, azw = rotate_vec_by_quat((ax, ay, az), q_render)

    # Resultant accel arrow in world frame
    ux, uy, uz, a_mag = v_norm(axw, ayw, azw)
    acc_arrow.axis = vector(ux, uy, uz) * (ACC_SCALE * a_mag)

    # -------------------------
    # Estimate "Up" direction from resultant accel direction (orientation agnostic)
    # -------------------------
    # During ascent/powered motion, measured accel tends to point along thrust/up direction (plus gravity).
    # Use the direction of a_world as a proxy for "up" and smooth it.
    if a_mag > 1.0:  # ignore very small magnitudes (noisy)
        # Keep u_up continuous (avoid sign flips)
        if v_dot(u_up, (ux, uy, uz)) < 0.0:
            ux, uy, uz = -ux, -uy, -uz
        u_up = v_lerp_unit(u_up, (ux, uy, uz), UP_AXIS_ALPHA)

    # Draw up axis arrow (purple)
    up_arrow.axis = vector(u_up[0], u_up[1], u_up[2]) * 1.2

    # -------------------------
    # Linear acceleration along "up"
    # -------------------------
    # Remove gravity vector projected onto u_up.
    # Gravity in world frame is (0,0,-G0) if Z is up. But we do NOT assume which axis is up globally here.
    # Instead: we treat gravity magnitude as G0 and remove its component along u_up by assuming gravity is opposite
    # the long-term average accel when stationary. Since we don't have that calibration here,
    # we approximate gravity projection as +G0 * cos(theta) where theta between u_up and gravity direction.
    #
    # Practical approach without magnetometer:
    # - Use the component of measured accel along u_up and subtract G0 (because when stationary,
    #   accel along any axis aligned with gravity is ~+G0).
    #
    # This works well for "move up then stop" tests and simple ascent/coast transitions.
    a_along_up = v_dot((axw, ayw, azw), u_up)   # includes gravity
    a_lin_up = a_along_up - G0                 # approximate remove 1g along that axis

    # Filter + integrate velocity (leaky to limit drift)
    nowt = time.time()
    dt = nowt - last_t
    last_t = nowt
    if dt <= 0 or dt > 0.2:
        dt = 1/60.0

    a_up_lp = a_up_lp + ACC_LP * (a_lin_up - a_up_lp)
    v_up = (1.0 - VEL_LEAK) * v_up + a_up_lp * dt

    # -------------------------
    # Apogee detection: after "armed" by clear upward velocity, trigger when v_up ~ 0
    # -------------------------
    if not armed and v_up > VEL_ARM:
        armed = True

    if armed and not apogee_detected:
        if v_up < VEL_APOGEE:
            if below_start is None:
                below_start = nowt
            elif (nowt - below_start) >= APOGEE_HOLD:
                apogee_detected = True
        else:
            below_start = None

    # Optional reset for repeated tests
    if apogee_detected and v_up > RESET_VEL:
        apogee_detected = False
        armed = True
        below_start = None

    # Update highlight boxes
    if apogee_detected:
        flight_box.opacity = 0.08
        apogee_box.opacity = 0.35
    else:
        flight_box.opacity = 0.35
        apogee_box.opacity = 0.08

    txt.text = (
        f"Port: {PORT}\n"
        f"Yaw {yaw:.1f}°  Pitch {pitch:.1f}°  Roll {roll:.1f}°\n"
        f"|a|={a_mag:.2f} m/s²   a_up={a_lin_up:+.2f} m/s²\n"
        f"v_up={v_up:+.2f} m/s   Armed={'YES' if armed else 'NO'}   Apogee={'YES' if apogee_detected else 'NO'}"
    )