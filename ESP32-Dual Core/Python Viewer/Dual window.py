import math
import time
import threading

import serial
import serial.tools.list_ports
from vpython import canvas, box, cone, arrow, vector, color, label, rate

# ============================================================
# SETTINGS
# ============================================================
BAUD = 115200
PORT = "COM9"          # set to None for auto-detect
STALE_TIMEOUT = 1.0
INIT_DURATION = 3.0

ARM_ACC_THRESH = 2.0
ARM_SPEED_THRESH = 1.0
APOGEE_SPEED_THRESH = 0.35
APOGEE_ACC_THRESH = 1.0
APOGEE_DWELL = 0.35
VEL_DAMPING_WHEN_COAST = 0.995

VECTOR_SCALE = 0.18

# Thin rectangle model — same size used in BOTH scenes
ROCKET_LEN = 2.7
ROCKET_THICK = 0.10
ROCKET_WIDTH = 0.08

BODY_AXIS_LEN = 1.0
WORLD_AXIS_LEN = 1.8

# ============================================================
# BMI160 → WORLD AXIS REMAP
# ============================================================
# BMI160 physical axes (from sensor image):
#   BMI_X  = up    (red arrow,   points away from board face)
#   BMI_Y  = left  (green arrow, points left on board)
#   BMI_Z  = right (blue arrow,  points right on board)
#
# Desired world mapping:
#   World Z  (up / rocket long axis) = +BMI_X
#   World X  (side tilt, roll)       = +BMI_Y
#   World Y  (forward / nose thrust) = -BMI_Z
#
# So to convert a raw sensor vector (bx, by, bz) to world:
#   wx =  by
#   wy = -bz
#   wz =  bx
def bmi_to_world(bx, by, bz):
    """Remap BMI160 sensor axes to visualiser world axes."""
    wx =  by    # side tilt
    wy = -bz    # forward/thrust (nose direction)
    wz =  bx    # up (rocket long axis)
    return wx, wy, wz


# ============================================================
# GLOBAL TELEMETRY STATE
# ============================================================
latest = {
    "q": (1.0, 0.0, 0.0, 0.0),
    "yaw": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 9.81,
    "amag": 9.81,
    "t": time.time(),
}

lock = threading.Lock()
running = True

# Calibration / init reference state
calibrating = True
calib_start = None
q_ref_sum = [0.0, 0.0, 0.0, 0.0]
q_ref_count = 0
q_init = (1.0, 0.0, 0.0, 0.0)

baseline_world_acc = vector(0, 0, 9.81)
baseline_world_acc_sum = vector(0, 0, 0)

# Motion state
vel_world = vector(0, 0, 0)
lin_world = vector(0, 0, 0)
lin_mag = 0.0
armed = False
apogee = False
apogee_timer = 0.0

# ============================================================
# SERIAL HELPERS
# ============================================================
def auto_pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    preferred = []
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        dev = (p.device or "").lower()
        if any(k in desc for k in ["cp210", "ch340", "usb serial", "uart", "silicon labs", "wch"]):
            preferred.append(p.device)
        elif "usb" in hwid or "usb" in dev:
            preferred.append(p.device)

    if preferred:
        return preferred[0]
    return ports[0].device


def open_serial():
    port_to_use = PORT if PORT else auto_pick_port()
    if not port_to_use:
        raise RuntimeError("No serial port found.")
    print(f"Opening serial port: {port_to_use} @ {BAUD}")
    return serial.Serial(port_to_use, BAUD, timeout=0.05)


# ============================================================
# QUATERNION HELPERS
# ============================================================
def q_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def q_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_rotate_vec(q, v):
    p = (0.0, v.x, v.y, v.z)
    qr = q_mul(q_mul(q, p), q_conj(q))
    return vector(qr[1], qr[2], qr[3])


def q_to_world_basis(q_rel):
    """
    Return the three world-space directions for the sensor body axes,
    applying the BMI160 → world remap so the visualiser body arrows
    match the physical sensor orientation.

    q_rel rotates sensor-body vectors into world space.
    We feed it the *remapped* sensor axes so the displayed arrows
    are consistent with the world frame defined by bmi_to_world().
    """
    # Remapped sensor unit vectors expressed in sensor-body frame
    sx_bmi = vector(0, 1, 0)   # BMI_Y  → world X
    sy_bmi = vector(0, 0, -1)  # -BMI_Z → world Y
    sz_bmi = vector(1, 0, 0)   # BMI_X  → world Z

    sx = q_rotate_vec(q_rel, sx_bmi)
    sy = q_rotate_vec(q_rel, sy_bmi)
    sz = q_rotate_vec(q_rel, sz_bmi)
    return sx, sy, sz


# ============================================================
# EULER-AXIS REMAP FOR DISPLAY
# ============================================================
# With the new axis convention the rocket's long axis is Z (up).
# We model the rectangle lying along +Y in object space and
# then apply the Euler angles remapped to:
#
#   pitch → rotation about world X  (nose up/down)
#   roll  → rotation about world Y  (side tilt)
#   yaw   → rotation about world Z  (heading spin)
#
# These are the BMI160 Euler outputs interpreted in the new frame.

def rot_x(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((1,0,0),(0,c,-s),(0,s,c))

def rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c,0,s),(0,1,0),(-s,0,c))

def rot_z(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c,-s,0),(s,c,0),(0,0,1))

def mat_mul(a, b):
    out = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k]*b[k][j] for k in range(3))
    return out

def mat_vec(m, v):
    return vector(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z,
    )


def rocket_axis_up_from_euler(yaw_deg, pitch_deg, roll_deg):
    """
    Compute the rocket long-axis direction and up-vector from Euler angles,
    using the remapped convention:
      pitch → Rx  (nose up/down  — tilts around world X)
      roll  → Ry  (side tilt     — tilts around world Y)
      yaw   → Rz  (heading spin  — rotates around world Z / up)

    The rectangle starts aligned with +Y (long axis) and +Z (up face).
    """
    Rx = rot_x(pitch_deg)
    Ry = rot_y(roll_deg)
    Rz = rot_z(yaw_deg)

    # Combined rotation: yaw last so heading is applied after tilt
    R = mat_mul(Rz, mat_mul(Ry, Rx))

    axis0 = vector(0, 1, 0)   # long axis of rocket in rest pose
    up0   = vector(0, 0, 1)   # "up" face of rectangle in rest pose

    axis = mat_vec(R, axis0)
    up   = mat_vec(R, up0)

    if axis.mag < 1e-9:
        axis = vector(0, 1, 0)
    if up.mag < 1e-9:
        up = vector(0, 0, 1)
    if abs(axis.norm().dot(up.norm())) > 0.98:
        up = vector(1, 0, 0)

    return axis.norm(), up.norm()


# ============================================================
# PARSING
# ============================================================
def strip_lora_prefix(line: str) -> str:
    s = line.strip()
    if s.startswith("RX#"):
        idx = s.find("DMPSTYLE,")
        if idx >= 0:
            return s[idx:]
    return s


def parse_dmpstyle(line: str):
    s = strip_lora_prefix(line)
    if not s.startswith("DMPSTYLE,"):
        return None

    parts = s.split(",")
    if len(parts) < 12:
        return None

    try:
        qw = float(parts[1])
        qx = float(parts[2])
        qy = float(parts[3])
        qz = float(parts[4])
        yaw   = float(parts[5])
        pitch = float(parts[6])
        roll  = float(parts[7])
        ax = float(parts[8])
        ay = float(parts[9])
        az = float(parts[10])
        amag = float(parts[11])

        # Remap raw sensor acceleration axes to world convention
        wx, wy, wz = bmi_to_world(ax, ay, az)

        return {
            "q":     q_normalize((qw, qx, qy, qz)),
            "yaw":   yaw,
            "pitch": pitch,
            "roll":  roll,
            # store remapped acceleration for physics integration
            "ax": wx,
            "ay": wy,
            "az": wz,
            "amag": amag,
            "t": time.time(),
        }
    except Exception:
        return None


# ============================================================
# SERIAL THREAD
# ============================================================
def serial_reader():
    global running
    ser = open_serial()

    while running:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            pkt = parse_dmpstyle(raw)
            if pkt is None:
                continue

            with lock:
                latest.update(pkt)

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            break
        except Exception as e:
            print(f"Reader warning: {e}")

    try:
        ser.close()
    except Exception:
        pass


# ============================================================
# SCENES
# ============================================================
scene_fixed = canvas(
    title="Apogee Detection + Resultant Vector (Fixed Model)",
    width=860,
    height=520,
    background=vector(0.95, 0.95, 0.95),
)

scene_rot = canvas(
    title="3D Rotation Viewer (Rotating Model)",
    width=860,
    height=520,
    background=vector(0.95, 0.95, 0.95),
)

# ---------------- Fixed scene ----------------
scene_fixed.forward = vector(-0.55, -0.35, -1.0)
scene_fixed.center = vector(0, 0.2, 0)
scene_fixed.range = 2.8
scene_fixed.userspin = False
scene_fixed.userzoom = True

fixed_body = box(
    canvas=scene_fixed,
    pos=vector(0, 0, 0),
    size=vector(ROCKET_THICK, ROCKET_LEN, ROCKET_WIDTH),
    color=vector(0.2, 0.2, 0.24),
)

fixed_nose = cone(
    canvas=scene_fixed,
    pos=vector(0, ROCKET_LEN / 2 + 0.14, 0),
    axis=vector(0, 0.36, 0),
    radius=0.10,
    color=color.orange,
)

fx = arrow(canvas=scene_fixed, pos=vector(0,0,0), axis=vector(WORLD_AXIS_LEN,0,0), shaftwidth=0.04, color=color.blue)
fy = arrow(canvas=scene_fixed, pos=vector(0,0,0), axis=vector(0,WORLD_AXIS_LEN,0), shaftwidth=0.04, color=color.green)
fz = arrow(canvas=scene_fixed, pos=vector(0,0,0), axis=vector(0,0,WORLD_AXIS_LEN), shaftwidth=0.04, color=color.red)

vector_arrow = arrow(
    canvas=scene_fixed,
    pos=vector(0,0,0),
    axis=vector(0,0,0),
    shaftwidth=0.05,
    color=color.cyan,
)

fixed_note = label(
    canvas=scene_fixed,
    pos=vector(0, 2.1, 0),
    text="FIXED MODEL VIEW  |  Upright original pose reference",
    box=False, opacity=0, height=14, color=color.black,
)

flight_label = label(
    canvas=scene_fixed,
    pos=vector(-1.75, 1.55, 0),
    text="IN FLIGHT",
    box=True, border=8, line=False, opacity=0.25,
    background=vector(0.85, 0.78, 0.78),
    color=color.black, height=18,
)

apogee_label = label(
    canvas=scene_fixed,
    pos=vector(1.75, 1.55, 0),
    text="APOGEE MODE",
    box=True, border=8, line=False, opacity=0.25,
    background=vector(0.47, 0.84, 0.44),
    color=color.black, height=18,
)

stats_label = label(
    canvas=scene_fixed,
    pos=vector(0, 1.05, 0),
    text="", box=False, opacity=0, height=14, color=color.black,
)

# ---------------- Rotating scene ----------------
# Camera looking slightly down-front so the upright rocket is visible
scene_rot.forward = vector(-0.55, -0.30, -1.0)
scene_rot.center = vector(0, 0.2, 0)
scene_rot.range = 2.8

# Rocket rectangle — SAME dimensions as the fixed scene
rot_body = box(
    canvas=scene_rot,
    pos=vector(0, 0, 0),
    size=vector(ROCKET_THICK, ROCKET_LEN, ROCKET_WIDTH),  # identical to fixed_body
    color=vector(0.2, 0.2, 0.24),
)

rot_nose = cone(
    canvas=scene_rot,
    pos=vector(0, ROCKET_LEN / 2 + 0.14, 0),
    axis=vector(0, 0.36, 0),
    radius=0.10,
    color=color.orange,
)

# World-frame reference arrows (white, thin)
rxw = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(WORLD_AXIS_LEN,0,0), shaftwidth=0.03, color=color.white)
ryw = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(0,WORLD_AXIS_LEN,0), shaftwidth=0.03, color=color.white)
rzw = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(0,0,WORLD_AXIS_LEN), shaftwidth=0.03, color=color.white)

# Body-frame arrows — coloured to match BMI160 diagram remapped convention:
#   X (blue)  = world X  ← BMI_Y  (side tilt)
#   Y (green) = world Y  ← -BMI_Z (forward/thrust)
#   Z (red)   = world Z  ← BMI_X  (up / long axis)
body_x = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(1,0,0), shaftwidth=0.04, color=color.blue)
body_y = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(0,1,0), shaftwidth=0.04, color=color.green)
body_z = arrow(canvas=scene_rot, pos=vector(0,0,0), axis=vector(0,0,1), shaftwidth=0.04, color=color.red)

body_x_label = label(canvas=scene_rot, pos=vector(1.15,0,0),    text="X", box=False, opacity=0, height=16, color=color.blue)
body_y_label = label(canvas=scene_rot, pos=vector(0,1.15,0),    text="Y", box=False, opacity=0, height=16, color=color.green)
body_z_label = label(canvas=scene_rot, pos=vector(0,0,1.15),    text="Z", box=False, opacity=0, height=16, color=color.red)

rot_stats = label(
    canvas=scene_rot,
    pos=vector(0, 2.1, 0),
    text="Rotation View",
    box=False, opacity=0, height=14, color=color.black,
)

# ============================================================
# CALIBRATION
# ============================================================
def accumulate_calibration(q_raw, a_body):
    global calib_start, q_ref_sum, q_ref_count, baseline_world_acc_sum

    now = time.time()
    if calib_start is None:
        calib_start = now

    qn = q_normalize(q_raw)

    if q_ref_count > 0:
        dot = (
            qn[0]*q_ref_sum[0] + qn[1]*q_ref_sum[1] +
            qn[2]*q_ref_sum[2] + qn[3]*q_ref_sum[3]
        )
        if dot < 0:
            qn = (-qn[0], -qn[1], -qn[2], -qn[3])

    q_ref_sum[0] += qn[0]
    q_ref_sum[1] += qn[1]
    q_ref_sum[2] += qn[2]
    q_ref_sum[3] += qn[3]
    q_ref_count  += 1

    a_world = q_rotate_vec(qn, a_body)
    baseline_world_acc_sum += a_world

    return now - calib_start


def finish_calibration():
    global calibrating, q_init, baseline_world_acc

    if q_ref_count <= 0:
        q_init = (1.0, 0.0, 0.0, 0.0)
        baseline_world_acc = vector(0, 0, 9.81)
    else:
        q_init = q_normalize((
            q_ref_sum[0] / q_ref_count,
            q_ref_sum[1] / q_ref_count,
            q_ref_sum[2] / q_ref_count,
            q_ref_sum[3] / q_ref_count,
        ))
        baseline_world_acc = baseline_world_acc_sum / q_ref_count

    calibrating = False
    print("Initialization complete.")
    print("q_init =", q_init)
    print("baseline_world_acc =", baseline_world_acc)


# ============================================================
# VISUALS
# ============================================================
def update_rotation_model(yaw_deg, pitch_deg, roll_deg, q_rel):
    """
    Orient the rotating rocket rectangle using the remapped Euler angles:
      pitch → Rx  (nose up/down)
      roll  → Ry  (side tilt)
      yaw   → Rz  (heading spin)

    Body-frame arrows use q_rel with the BMI→world basis vectors so
    they faithfully represent the sensor orientation in world space.
    """
    rocket_axis, rocket_up = rocket_axis_up_from_euler(yaw_deg, pitch_deg, roll_deg)

    rot_body.axis = rocket_axis * ROCKET_LEN
    rot_body.up   = rocket_up

    rot_nose.pos  = rot_body.pos + rocket_axis * (ROCKET_LEN / 2 + 0.14)
    rot_nose.axis = rocket_axis * 0.36
    rot_nose.up   = rocket_up

    sx, sy, sz = q_to_world_basis(q_rel)

    body_x.pos = vector(0, 0, 0);  body_x.axis = sx * BODY_AXIS_LEN
    body_y.pos = vector(0, 0, 0);  body_y.axis = sy * BODY_AXIS_LEN
    body_z.pos = vector(0, 0, 0);  body_z.axis = sz * BODY_AXIS_LEN

    body_x_label.pos = body_x.pos + body_x.axis * 1.12
    body_y_label.pos = body_y.pos + body_y.axis * 1.12
    body_z_label.pos = body_z.pos + body_z.axis * 1.12


def update_fixed_vector(lw):
    vector_arrow.pos  = vector(0, 0, 0)
    vector_arrow.axis = lw * VECTOR_SCALE


# ============================================================
# MAIN
# ============================================================
def main():
    global running, vel_world, lin_world, lin_mag, armed, apogee, apogee_timer

    reader = threading.Thread(target=serial_reader, daemon=True)
    reader.start()

    prev_t = time.time()

    while True:
        rate(100)

        now = time.time()
        dt = max(0.001, min(0.05, now - prev_t))
        prev_t = now

        with lock:
            pkt = latest.copy()

        age   = now - pkt["t"]
        stale = age > STALE_TIMEOUT

        q_raw  = q_normalize(pkt["q"])
        # a_body is already remapped to world convention by parse_dmpstyle
        a_body = vector(pkt["ax"], pkt["ay"], pkt["az"])

        if calibrating:
            elapsed = accumulate_calibration(q_raw, a_body)
            remain  = max(0.0, INIT_DURATION - elapsed)

            stats_label.text = (
                f"Port: {PORT or 'AUTO'}\n"
                f"INITIALISING... {remain:0.1f}s\n"
                f"Hold the sensor in the ORIGINAL upright position"
            )
            rot_stats.text = f"INITIALISING... {remain:0.1f}s"
            vector_arrow.axis = vector(0, 0, 0)

            if elapsed >= INIT_DURATION:
                finish_calibration()
            continue

        # Relative orientation from original init pose
        q_rel = q_mul(q_conj(q_init), q_raw)
        q_rel = q_normalize(q_rel)

        # Linear acceleration in world frame (gravity subtracted)
        a_world  = q_rotate_vec(q_rel, a_body)
        lin_world = a_world - baseline_world_acc
        lin_mag   = lin_world.mag

        vel_world += lin_world * dt

        if lin_mag < 0.25:
            vel_world *= VEL_DAMPING_WHEN_COAST

        speed = vel_world.mag

        if not armed and (lin_mag >= ARM_ACC_THRESH or speed >= ARM_SPEED_THRESH):
            armed = True
            apogee = False
            apogee_timer = 0.0

        if armed and not apogee:
            if speed <= APOGEE_SPEED_THRESH and lin_mag <= APOGEE_ACC_THRESH:
                apogee_timer += dt
                if apogee_timer >= APOGEE_DWELL:
                    apogee = True
            else:
                apogee_timer = 0.0

        yaw_disp   = pkt["yaw"]
        pitch_disp = pkt["pitch"]
        roll_disp  = pkt["roll"]

        update_rotation_model(yaw_disp, pitch_disp, roll_disp, q_rel)
        update_fixed_vector(lin_world)

        if stale:
            rot_stats.text  = "STALE DATA"
            stats_label.text = "STALE DATA"
        else:
            stats_label.text = (
                f"Port: {PORT or 'AUTO'}\n"
                f"Yaw {yaw_disp:0.1f}°  Pitch {pitch_disp:0.1f}°  Roll {roll_disp:0.1f}°\n"
                f"|a|={pkt['amag']:0.2f} m/s²   |a_lin|={lin_mag:0.2f} m/s²   |v|={speed:0.2f} m/s\n"
                f"Armed={'YES' if armed else 'NO'}   Apogee={'YES' if apogee else 'NO'}"
            )
            rot_stats.text = (
                f"Yaw {yaw_disp:0.1f}°   Pitch {pitch_disp:0.1f}°   Roll {roll_disp:0.1f}°\n"
                f"q_rel = ({q_rel[0]:0.3f}, {q_rel[1]:0.3f}, {q_rel[2]:0.3f}, {q_rel[3]:0.3f})"
            )

        if armed and not apogee:
            flight_label.background = vector(1.0, 0.40, 0.40)
        else:
            flight_label.background = vector(0.85, 0.78, 0.78)

        if apogee:
            apogee_label.background = vector(0.20, 0.85, 0.20)
        else:
            apogee_label.background = vector(0.47, 0.84, 0.44)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("Exiting...")