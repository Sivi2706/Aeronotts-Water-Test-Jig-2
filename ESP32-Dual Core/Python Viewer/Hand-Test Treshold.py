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

# Hand-test-friendly launch / apogee detection based on
# DELTA from startup raw acceleration magnitude
DELTA_LAUNCH_THRESH = 0.8    # m/s^2, movement away from startup value -> in flight
DELTA_APOGEE_THRESH = 0.7    # m/s^2, returned near startup value -> apogee candidate
APOGEE_DWELL = 0.12          # s, must remain near startup value this long

# Reset logic so apogee does not stay latched forever
RESET_DELTA_THRESH = 0.35    # m/s^2, near-baseline quiet zone
RESET_DWELL = 0.60           # s, stay quiet this long to return to IDLE

VECTOR_SCALE = 0.18
CSV_LOG_HZ = 10.0

# Rocket model dimensions — same in BOTH scenes
ROCKET_LEN   = 2.7
ROCKET_THICK = 0.10
ROCKET_WIDTH = 0.08

BODY_AXIS_LEN  = 1.0
WORLD_AXIS_LEN = 1.8

# ============================================================
# BMI160 -> WORLD AXIS REMAP
# ============================================================
def bmi_to_world(bx, by, bz):
    return bx, by, bz


# ============================================================
# GLOBAL TELEMETRY STATE
# ============================================================
latest = {
    "q":    (1.0, 0.0, 0.0, 0.0),
    "yaw":  0.0, "pitch": 0.0, "roll": 0.0,
    "ax":   0.0, "ay":    0.0, "az":   9.81,
    "amag": 9.81,
    "t":    time.time(),
}

lock = threading.Lock()
running = True

# Calibration state
calibrating        = True
calib_start        = None
q_ref_sum          = [0.0, 0.0, 0.0, 0.0]
q_ref_count        = 0
q_init             = (1.0, 0.0, 0.0, 0.0)

# Baseline accel vector in the initialization reference frame
baseline_ref_acc     = vector(0, 0, 9.81)
baseline_ref_acc_sum = vector(0, 0, 0)

# Startup raw acceleration magnitude baseline
baseline_raw_mag     = 9.81
baseline_raw_mag_sum = 0.0

# Flight / state machine
STATE_IDLE = 0
STATE_IN_FLIGHT = 1
STATE_APOGEE = 2

flight_state = STATE_IDLE
apogee_timer = 0.0
reset_timer = 0.0

# Kinematics display
lin_world = vector(0, 0, 0)
lin_mag = 0.0

# Logging
last_csv_log_t = 0.0

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
        dev  = (p.device or "").lower()
        if any(k in desc for k in ["cp210", "ch340", "usb serial", "uart", "silicon labs", "wch"]):
            preferred.append(p.device)
        elif "usb" in hwid or "usb" in dev:
            preferred.append(p.device)
    return preferred[0] if preferred else ports[0].device


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
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)


def q_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def q_rotate_vec(q, v):
    p = (0.0, v.x, v.y, v.z)
    qr = q_mul(q_mul(q, p), q_conj(q))
    return vector(qr[1], qr[2], qr[3])


def q_remap_bmi_to_world(q):
    return q_normalize(q)


def q_to_world_basis(q):
    """
    Rotate the three world unit vectors by q, then convert to VPython display space.
    world(x, y, z) -> vpython(z, x, y)
    """
    wx = q_rotate_vec(q, vector(1, 0, 0))  # world X (up)
    wy = q_rotate_vec(q, vector(0, 1, 0))  # world Y (side)
    wz = q_rotate_vec(q, vector(0, 0, 1))  # world Z (forward)

    sx = vector(wx.z, wx.x, wx.y)
    sy = vector(wy.z, wy.x, wy.y)
    sz = vector(wz.z, wz.x, wz.y)
    return sx, sy, sz


def rocket_axis_up_from_qrel(q_remapped):
    """
    The 3-second initialization pose becomes the upright reference.
    After init:
        q_rel = conj(q_init) * q_raw
    so q_rel = identity means "same as initialized pose".
    """
    w_axis = q_rotate_vec(q_remapped, vector(1, 0, 0))
    w_face = q_rotate_vec(q_remapped, vector(0, 0, 1))

    # world -> VPython
    axis = vector(w_axis.z, w_axis.x, w_axis.y)
    up   = vector(w_face.z, w_face.x, w_face.y)

    if axis.mag < 1e-9:
        axis = vector(0, 1, 0)
    if up.mag < 1e-9:
        up = vector(1, 0, 0)
    if abs(axis.norm().dot(up.norm())) > 0.98:
        up = vector(0, 0, 1)

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
        qw    = float(parts[1]); qx    = float(parts[2])
        qy    = float(parts[3]); qz    = float(parts[4])
        yaw   = float(parts[5]); pitch = float(parts[6]); roll = float(parts[7])
        ax    = float(parts[8]); ay    = float(parts[9]); az   = float(parts[10])
        amag  = float(parts[11])

        wx, wy, wz = bmi_to_world(ax, ay, az)

        return {
            "q":    q_normalize((qw, qx, qy, qz)),
            "yaw":  yaw,
            "pitch": pitch,
            "roll": roll,
            "ax":   wx,
            "ay":   wy,
            "az":   wz,
            "amag": amag,
            "t":    time.time(),
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
    width=860, height=520,
    background=vector(0.95, 0.95, 0.95),
)
scene_fixed.forward  = vector(-0.4, -0.2, -1.0)
scene_fixed.up       = vector(0, 1, 0)
scene_fixed.center   = vector(0, 0, 0)
scene_fixed.range    = 2.8
scene_fixed.userspin = False
scene_fixed.userzoom = True

scene_rot = canvas(
    title="3D Rotation Viewer (Init Pose = Absolute Upright Reference)",
    width=860, height=520,
    background=vector(0.95, 0.95, 0.95),
)
scene_rot.forward = vector(-0.4, -0.2, -1.0)
scene_rot.up      = vector(0, 1, 0)
scene_rot.center  = vector(0, 0, 0)
scene_rot.range   = 2.8

# ── Fixed scene ───────────────────────────────────────────────
fixed_body = box(
    canvas=scene_fixed, pos=vector(0, 0, 0),
    size=vector(ROCKET_LEN, ROCKET_THICK, ROCKET_WIDTH),
    axis=vector(0, 1, 0), up=vector(0, 0, 1),
    color=vector(0.2, 0.2, 0.24),
)
fixed_nose = cone(
    canvas=scene_fixed,
    pos=vector(0, ROCKET_LEN/2 + 0.14, 0),
    axis=vector(0, 0.36, 0), radius=0.10,
    color=color.orange,
)

fx = arrow(canvas=scene_fixed, pos=vector(0, 0, 0),
           axis=vector(0, WORLD_AXIS_LEN, 0), shaftwidth=0.04, color=color.blue)
fy = arrow(canvas=scene_fixed, pos=vector(0, 0, 0),
           axis=vector(0, 0, WORLD_AXIS_LEN), shaftwidth=0.04, color=color.green)
fz = arrow(canvas=scene_fixed, pos=vector(0, 0, 0),
           axis=vector(WORLD_AXIS_LEN, 0, 0), shaftwidth=0.04, color=color.red)

fx_label = label(canvas=scene_fixed, pos=vector(0, WORLD_AXIS_LEN+0.22, 0),
                 text="X  (up)", box=False, opacity=1, height=14,
                 color=color.blue, bold=True)
fy_label = label(canvas=scene_fixed, pos=vector(0, 0, WORLD_AXIS_LEN+0.22),
                 text="Y  (side)", box=False, opacity=1, height=14,
                 color=color.green, bold=True)
fz_label = label(canvas=scene_fixed, pos=vector(WORLD_AXIS_LEN+0.22, 0, 0),
                 text="Z  (forward)", box=False, opacity=1, height=14,
                 color=color.red, bold=True)

vector_arrow = arrow(
    canvas=scene_fixed, pos=vector(0, 0, 0),
    axis=vector(0, 0, 0), shaftwidth=0.05, color=color.cyan,
)

fixed_note = label(
    canvas=scene_fixed, pos=vector(0, 2.1, 0),
    text="FIXED MODEL VIEW  |  Cyan arrow = resultant linear acceleration vector",
    box=False, opacity=0, height=14, color=color.black,
)

flight_label = label(
    canvas=scene_fixed, pos=vector(-1.75, 1.55, 0),
    text="IN FLIGHT", box=True, border=8, line=False, opacity=0.25,
    background=vector(0.85, 0.78, 0.78), color=color.black, height=18,
)
apogee_label = label(
    canvas=scene_fixed, pos=vector(1.75, 1.55, 0),
    text="APOGEE MODE", box=True, border=8, line=False, opacity=0.25,
    background=vector(0.47, 0.84, 0.44), color=color.black, height=18,
)
stats_label = label(
    canvas=scene_fixed, pos=vector(0, -2.05, 0),
    text="", box=False, opacity=0, height=14, color=color.black,
)

# ── Rotating scene ────────────────────────────────────────────
rot_body = box(
    canvas=scene_rot, pos=vector(0, 0, 0),
    size=vector(ROCKET_LEN, ROCKET_THICK, ROCKET_WIDTH),
    axis=vector(0, 1, 0), up=vector(0, 0, 1),
    color=vector(0.2, 0.2, 0.24),
)
rot_nose = cone(
    canvas=scene_rot,
    pos=vector(0, ROCKET_LEN/2 + 0.14, 0),
    axis=vector(0, 0.36, 0), radius=0.10,
    color=color.orange,
)

rxw = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
            axis=vector(0, WORLD_AXIS_LEN, 0), shaftwidth=0.03, color=color.white)
ryw = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
            axis=vector(0, 0, WORLD_AXIS_LEN), shaftwidth=0.03, color=color.white)
rzw = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
            axis=vector(WORLD_AXIS_LEN, 0, 0), shaftwidth=0.03, color=color.white)

rxw_label = label(canvas=scene_rot, pos=vector(0, WORLD_AXIS_LEN+0.22, 0),
                  text="X  (up)", box=False, opacity=1, height=13,
                  color=color.white, bold=True)
ryw_label = label(canvas=scene_rot, pos=vector(0, 0, WORLD_AXIS_LEN+0.22),
                  text="Y  (side)", box=False, opacity=1, height=13,
                  color=color.white, bold=True)
rzw_label = label(canvas=scene_rot, pos=vector(WORLD_AXIS_LEN+0.22, 0, 0),
                  text="Z  (forward)", box=False, opacity=1, height=13,
                  color=color.white, bold=True)

body_x = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
               axis=vector(0, BODY_AXIS_LEN, 0), shaftwidth=0.04, color=color.blue)
body_y = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
               axis=vector(0, 0, BODY_AXIS_LEN), shaftwidth=0.04, color=color.green)
body_z = arrow(canvas=scene_rot, pos=vector(0, 0, 0),
               axis=vector(BODY_AXIS_LEN, 0, 0), shaftwidth=0.04, color=color.red)

body_x_label = label(canvas=scene_rot, pos=vector(0, BODY_AXIS_LEN*1.15, 0),
                     text="X", box=False, opacity=1, height=16,
                     color=color.blue, bold=True)
body_y_label = label(canvas=scene_rot, pos=vector(0, 0, BODY_AXIS_LEN*1.15),
                     text="Y", box=False, opacity=1, height=16,
                     color=color.green, bold=True)
body_z_label = label(canvas=scene_rot, pos=vector(BODY_AXIS_LEN*1.15, 0, 0),
                     text="Z", box=False, opacity=1, height=16,
                     color=color.red, bold=True)

rot_stats = label(
    canvas=scene_rot, pos=vector(0, -2.0, 0),
    text="Rotation View", box=False, opacity=0, height=14, color=color.black,
)

# ============================================================
# CALIBRATION
# ============================================================
def accumulate_calibration(q_raw, a_body):
    global calib_start, q_ref_sum, q_ref_count
    global baseline_ref_acc_sum, baseline_raw_mag_sum

    now = time.time()
    if calib_start is None:
        calib_start = now

    qn = q_normalize(q_raw)

    # Keep quaternion averaging sign-consistent
    if q_ref_count > 0:
        dot = (
            qn[0]*q_ref_sum[0] +
            qn[1]*q_ref_sum[1] +
            qn[2]*q_ref_sum[2] +
            qn[3]*q_ref_sum[3]
        )
        if dot < 0:
            qn = (-qn[0], -qn[1], -qn[2], -qn[3])

    q_ref_sum[0] += qn[0]
    q_ref_sum[1] += qn[1]
    q_ref_sum[2] += qn[2]
    q_ref_sum[3] += qn[3]
    q_ref_count += 1

    baseline_ref_acc_sum += a_body
    baseline_raw_mag_sum += a_body.mag

    return now - calib_start


def finish_calibration():
    global calibrating, q_init, baseline_ref_acc, baseline_raw_mag

    if q_ref_count <= 0:
        q_init = (1.0, 0.0, 0.0, 0.0)
        baseline_ref_acc = vector(0, 0, 9.81)
        baseline_raw_mag = 9.81
    else:
        q_init = q_normalize((
            q_ref_sum[0] / q_ref_count,
            q_ref_sum[1] / q_ref_count,
            q_ref_sum[2] / q_ref_count,
            q_ref_sum[3] / q_ref_count,
        ))
        baseline_ref_acc = baseline_ref_acc_sum / q_ref_count
        baseline_raw_mag = baseline_raw_mag_sum / q_ref_count

    calibrating = False
    print("Initialisation complete.")
    print("q_init =", q_init)
    print("baseline_ref_acc =", baseline_ref_acc)
    print("baseline_raw_mag =", baseline_raw_mag)
    print("CSV_HEADER,time_s,ax_lin,ay_lin,az_lin,a_lin_mag,raw_a_mag,delta_a_mag,state")


# ============================================================
# VISUALS
# ============================================================
def update_rotation_model(q_rel):
    q_disp = q_remap_bmi_to_world(q_rel)
    rocket_axis, rocket_up = rocket_axis_up_from_qrel(q_disp)

    rot_body.axis = rocket_axis * ROCKET_LEN
    rot_body.up   = rocket_up
    rot_nose.pos  = rot_body.pos + rocket_axis * (ROCKET_LEN/2 + 0.14)
    rot_nose.axis = rocket_axis * 0.36
    rot_nose.up   = rocket_up

    sx, sy, sz = q_to_world_basis(q_disp)
    body_x.axis = sx * BODY_AXIS_LEN
    body_y.axis = sy * BODY_AXIS_LEN
    body_z.axis = sz * BODY_AXIS_LEN

    body_x_label.pos = body_x.pos + body_x.axis * 1.15
    body_y_label.pos = body_y.pos + body_y.axis * 1.15
    body_z_label.pos = body_z.pos + body_z.axis * 1.15


def update_fixed_vector(lw):
    vector_arrow.pos = vector(0, 0, 0)
    vector_arrow.axis = lw * VECTOR_SCALE


# ============================================================
# LOGGING
# ============================================================
def state_name(s):
    if s == STATE_IDLE:
        return "IDLE"
    if s == STATE_IN_FLIGHT:
        return "IN_FLIGHT"
    if s == STATE_APOGEE:
        return "APOGEE"
    return "UNKNOWN"


def maybe_log_csv(now_s, lin_vec, lin_mag_val, raw_mag_val, delta_mag_val, state_val):
    global last_csv_log_t
    if now_s - last_csv_log_t < (1.0 / CSV_LOG_HZ):
        return

    last_csv_log_t = now_s
    print(
        f"DATA,{now_s:.3f},"
        f"{lin_vec.x:.4f},{lin_vec.y:.4f},{lin_vec.z:.4f},"
        f"{lin_mag_val:.4f},{raw_mag_val:.4f},{delta_mag_val:.4f},"
        f"{state_name(state_val)}"
    )


# ============================================================
# MAIN
# ============================================================
def main():
    global running, lin_world, lin_mag
    global flight_state, apogee_timer, reset_timer

    reader = threading.Thread(target=serial_reader, daemon=True)
    reader.start()

    start_time = time.time()
    prev_t = time.time()

    while True:
        rate(100)

        now = time.time()
        dt = max(0.001, min(0.05, now - prev_t))
        prev_t = now

        with lock:
            pkt = latest.copy()

        age = now - pkt["t"]
        stale = age > STALE_TIMEOUT

        q_raw = q_normalize(pkt["q"])
        a_body = vector(pkt["ax"], pkt["ay"], pkt["az"])

        if calibrating:
            elapsed = accumulate_calibration(q_raw, a_body)
            remain = max(0.0, INIT_DURATION - elapsed)

            stats_label.text = (
                f"Port: {PORT or 'AUTO'}\n"
                f"INITIALISING... {remain:0.1f}s\n"
                f"Hold the sensor in the ABSOLUTE UPRIGHT reference position"
            )
            rot_stats.text = (
                f"INITIALISING... {remain:0.1f}s\n"
                f"This pose becomes the upright reference"
            )
            vector_arrow.axis = vector(0, 0, 0)

            if elapsed >= INIT_DURATION:
                finish_calibration()
            continue

        # Relative orientation from init pose
        q_rel = q_normalize(q_mul(q_conj(q_init), q_raw))

        # Rotate current accel into init reference frame and subtract startup vector baseline
        a_ref = q_rotate_vec(q_rel, a_body)
        lin_world = a_ref - baseline_ref_acc
        lin_mag = lin_world.mag

        # Delta from startup raw magnitude for hand-test launch/apogee detection
        raw_mag = a_body.mag
        delta_mag = abs(raw_mag - baseline_raw_mag)

        # ------------------------------------------------------------
        # STATE MACHINE
        # ------------------------------------------------------------
        if stale:
            apogee_timer = 0.0
            reset_timer = 0.0
        else:
            if flight_state == STATE_IDLE:
                if delta_mag >= DELTA_LAUNCH_THRESH:
                    flight_state = STATE_IN_FLIGHT
                    apogee_timer = 0.0
                    reset_timer = 0.0

            elif flight_state == STATE_IN_FLIGHT:
                if delta_mag <= DELTA_APOGEE_THRESH:
                    apogee_timer += dt
                    if apogee_timer >= APOGEE_DWELL:
                        flight_state = STATE_APOGEE
                        reset_timer = 0.0
                else:
                    apogee_timer = 0.0

            elif flight_state == STATE_APOGEE:
                # Auto-reset once the sensor settles quietly near baseline
                if delta_mag <= RESET_DELTA_THRESH:
                    reset_timer += dt
                    if reset_timer >= RESET_DWELL:
                        flight_state = STATE_IDLE
                        apogee_timer = 0.0
                        reset_timer = 0.0
                else:
                    # If it starts moving again strongly, allow immediate relaunch
                    if delta_mag >= DELTA_LAUNCH_THRESH:
                        flight_state = STATE_IN_FLIGHT
                        apogee_timer = 0.0
                        reset_timer = 0.0
                    else:
                        reset_timer = 0.0

        yaw_disp = pkt["yaw"]
        pitch_disp = pkt["pitch"]
        roll_disp = pkt["roll"]

        update_rotation_model(q_rel)
        update_fixed_vector(lin_world)

        maybe_log_csv(
            now - start_time,
            lin_world,
            lin_mag,
            raw_mag,
            delta_mag,
            flight_state
        )

        if stale:
            rot_stats.text = "STALE DATA"
            stats_label.text = "STALE DATA"
        else:
            stats_label.text = (
                f"Port: {PORT or 'AUTO'}\n"
                f"Yaw {yaw_disp:0.1f}  Pitch {pitch_disp:0.1f}  Roll {roll_disp:0.1f}\n"
                f"Raw |a|={raw_mag:0.2f} m/s²   Startup |a|={baseline_raw_mag:0.2f} m/s²\n"
                f"Delta |a|={delta_mag:0.2f} m/s²\n"
                f"Resultant linear accel vector = ({lin_world.x:0.2f}, {lin_world.y:0.2f}, {lin_world.z:0.2f}) m/s²\n"
                f"|a_lin|={lin_mag:0.2f} m/s²\n"
                f"State={state_name(flight_state)}"
            )

            rot_stats.text = (
                f"Init pose = upright reference\n"
                f"Yaw {yaw_disp:0.1f}   Pitch {pitch_disp:0.1f}   Roll {roll_disp:0.1f}\n"
                f"q_rel = ({q_rel[0]:0.3f}, {q_rel[1]:0.3f}, {q_rel[2]:0.3f}, {q_rel[3]:0.3f})\n"
                f"State = {state_name(flight_state)}"
            )

        # Light boxes
        flight_label.background = (
            vector(1.0, 0.25, 0.25)
            if (flight_state == STATE_IN_FLIGHT)
            else vector(0.85, 0.78, 0.78)
        )

        apogee_label.background = (
            vector(0.10, 0.90, 0.10)
            if (flight_state == STATE_APOGEE)
            else vector(0.47, 0.84, 0.44)
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        running = False
        print("Exiting...")