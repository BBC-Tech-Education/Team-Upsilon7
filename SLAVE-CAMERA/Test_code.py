# =============================================================================
# victim_reader_uart.py  (v2)
# OpenMV AE3 - Cognitive-target + letter victim reader + UART reporter
# -----------------------------------------------------------------------------
# New in v2:
#   * DEBUG_VISION  : pure vision loop (no UART) to validate/tune the ring read
#   * MCU_PRESENT   : when False, the link self-acks so you can test the full
#                     STOP->VICTIM->GO protocol with nothing connected
#   * classify_letter: REAL FOMO inference via the ml module (not saved images)
#
# Protocol (OpenMV -> MCU):  [0xAA, TYPE, SEQ, CHK]   CHK=(TYPE^SEQ)&0xFF
#   CONNECT=0x01 STOP=0x02 GO=0x03 UNHARMED=0x04 STABLE=0x05 HARMED=0x06
# Ack (MCU -> OpenMV):       [0x55, SEQ, CHK]         CHK=(0x55^SEQ)&0xFF
#   MCU acts on each SEQ once, validates CHK, does NOT ack CONNECT.
# =============================================================================

import sensor
import image
import time
import math
import tof

from machine import UART

try:
    import ml
except Exception:
    ml = None

print("RUNNING victim_reader_uart.py v2")

# =============================================================================
# DEBUG / MODE SWITCHES
# =============================================================================

DEBUG_VISION = True     # True -> pure vision loop, no UART, rich overlay/print
MCU_PRESENT = True       # False -> link self-acks (test protocol w/o MCU)
SIM_ACK_MS = 60          # simulated ack latency when MCU_PRESENT is False
SHOW_DEBUG = True        # print state transitions
CALIBRATE_COLORS = False # True -> print centre-ROI LAB only (colour calibration)

# =============================================================================
# CALIBRATION / TUNING
# =============================================================================

CAL_K = 3870.0           # r0 = CAL_K / (d_tof - CAL_D0)
CAL_D0 = 14.0
R_PX_MIN = 30
R_PX_MAX = 130

RING_FRACS = [0.11, 0.30, 0.50, 0.70, 0.89]
FULL_ANGLES = [15, 45, 75, 105, 135, 165, 195, 225, 255, 285, 315, 345]
INNER_ANGLES = [30, 90, 150, 210, 270, 330]
ROI_FRAC = 0.11
MIN_VOTES = 3
MIN_CONF_STRICT = 0.50
R_SCAN = [0.92, 0.96, 1.00, 1.04, 1.08]

DISC_MIN_DIM = 50
DISC_MAX_DIM = 240
DISC_MIN_PIXELS = 300
ASPECT_MIN = 0.78
ASPECT_MAX = 1.28
RADIUS_AGREE_FRAC = 0.45

SUSPECT_HITS = 2
CONFIRM_MATCHES = 3
READ_TIMEOUT_MS = 1500
COOLDOWN_MS = 1500
SEND_STOP = True

# ---- Colour model (CALIBRATE under locked exposure + LED) ----
DISC_THRESHOLDS = [
    (0, 25, -20, 20, -20, 20),
    (20, 70, 25, 70, 5, 60),
    (60, 100, -25, 15, 30, 90),
    (25, 75, -70, -10, -5, 60),
    (10, 60, -15, 40, -75, -10),
]
COLOR_REFS = {
    "BLACK":  (10.0, 0.0, 0.0),
    "RED":    (26.0, 20.0, 17.0),
    "YELLOW": (69.0, -8.0, 33.0),
    "GREEN":  (16.0, -14.0, 8.0),
    "BLUE":   (40.0, -6.0, -23.0),
}
COLOR_MAXDIST2 = 1600.0
VALUES = {"BLACK": -2, "RED": -1, "YELLOW": 0, "GREEN": 1, "BLUE": 2}

# ---- Letter FOMO model ----
# Deploy your Edge Impulse FOMO model to the camera (copy trained.tflite +
# labels). Set the path and the channel order EXACTLY as the model outputs
# (FOMO adds a 'background' class). LETTER_TO_VICTIM maps each Greek letter
# label to a victim type.
LETTER_ENABLED = False
LETTER_MODEL_PATH = "trained.tflite"
LETTER_LABELS_ORDER = ["background", "phi", "psi", "omega"]  # MATCH your model
LETTER_BG_LABEL = "background"
LETTER_TO_VICTIM = {"phi": "HARMED", "psi": "STABLE", "omega": "UNHARMED"}
LETTER_CONF = 0.60       # min per-class max-cell probability to accept

# =============================================================================
# PROTOCOL BYTES
# =============================================================================

UART_ID = 1
UART_BAUD = 115200
RETRY_MS = 50
STOP_TIMEOUT_MS = 500
VICTIM_TIMEOUT_MS = 600
GO_TIMEOUT_MS = 2000
HEARTBEAT_MS = 200

START = 0xAA
ACK = 0x55
T_CONNECT = 0x01
T_STOP = 0x02
T_GO = 0x03
T_UNHARMED = 0x04
T_STABLE = 0x05
T_HARMED = 0x06
VICTIM_BYTE = {"UNHARMED": T_UNHARMED, "STABLE": T_STABLE, "HARMED": T_HARMED}

# =============================================================================
# UART LINK
# =============================================================================


class Link:

    def __init__(self, uart, retry_ms, hb_ms):
        self.uart = uart
        self.retry_ms = retry_ms
        self.hb_ms = hb_ms
        self.seq = 0
        self.hb_seq = 0
        self.pending = False
        self.p_type = 0
        self.p_seq = 0
        self.p_start = 0
        self.deadline = 0
        self.last_send = 0
        self.last_hb = time.ticks_ms()
        self.rxbuf = bytearray()
        self.sim_ack = False          # self-ack for MCU-less testing

    def _send(self, mtype, seq):
        chk = (mtype ^ seq) & 0xFF
        self.uart.write(bytes([START, mtype, seq, chk]))

    def start_msg(self, mtype, timeout_ms):
        self.seq = (self.seq + 1) & 0xFF
        if self.seq == 0:
            self.seq = 1
        now = time.ticks_ms()
        self.pending = True
        self.p_type = mtype
        self.p_seq = self.seq
        self.p_start = now
        self.deadline = time.ticks_add(now, timeout_ms)
        self._send(mtype, self.seq)
        self.last_send = now
        return self.seq

    def _pump_rx(self):
        n = self.uart.any()
        if n:
            data = self.uart.read(n)
            if data:
                self.rxbuf.extend(data)
        if len(self.rxbuf) > 64:
            self.rxbuf = self.rxbuf[-8:]

        acked = []
        buf = self.rxbuf
        L = len(buf)
        i = 0
        while True:
            while i < L and buf[i] != ACK:
                i += 1
            if i + 3 > L:
                break
            seq = buf[i + 1]
            chk = buf[i + 2]
            if ((ACK ^ seq) & 0xFF) == chk:
                acked.append(seq)
                i += 3
            else:
                i += 1
        self.rxbuf = bytearray(buf[i:])
        return acked

    def service(self, now):
        acked = self._pump_rx()

        # inject a simulated ack when no MCU is connected
        if self.sim_ack and self.pending:
            if time.ticks_diff(now, self.p_start) >= SIM_ACK_MS:
                acked.append(self.p_seq)

        if not self.pending:
            return None
        if self.p_seq in acked:
            self.pending = False
            return "acked"
        if time.ticks_diff(now, self.deadline) >= 0:
            self.pending = False
            return "timeout"
        if time.ticks_diff(now, self.last_send) >= self.retry_ms:
            self._send(self.p_type, self.p_seq)
            self.last_send = now
        return "pending"

    def heartbeat(self, now):
        if time.ticks_diff(now, self.last_hb) >= self.hb_ms:
            self.hb_seq = (self.hb_seq + 1) & 0xFF
            self._send(T_CONNECT, self.hb_seq)
            self.last_hb = now

# =============================================================================
# HARDWARE INIT
# =============================================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

sensor.set_auto_whitebal(True)
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
for _ in range(80):
    sensor.snapshot()
sensor.set_auto_whitebal(False)
for _ in range(30):
    sensor.snapshot()
_exp = sensor.get_exposure_us()
_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db=_gain)
sensor.set_auto_exposure(False, exposure_us=_exp)
print("Locked exposure_us =", _exp, " gain_db =", _gain)

tof.init()
TOF_W = tof.width()
TOF_H = tof.height()
print("ToF grid: %dx%d  refresh=%dHz" % (TOF_W, TOF_H, tof.refresh()))

IMG_W = sensor.width()
IMG_H = sensor.height()

uart = UART(UART_ID, UART_BAUD)
try:
    uart.init(UART_BAUD, bits=8, parity=None, stop=1, timeout=0, timeout_char=0)
except Exception:
    pass
link = Link(uart, RETRY_MS, HEARTBEAT_MS)
link.sim_ack = (not MCU_PRESENT)

# Load FOMO letter model once (optional)
letter_model = None
if LETTER_ENABLED and ml is not None:
    try:
        letter_model = ml.Model(LETTER_MODEL_PATH)
        print("Letter model loaded:", LETTER_MODEL_PATH, "ram=", letter_model.ram)
    except Exception as e:
        print("Letter model load FAILED:", e)
        letter_model = None

clock = time.clock()

# =============================================================================
# HELPERS
# =============================================================================


def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return None
    m = n // 2
    if n % 2 == 1:
        return s[m]
    return (s[m - 1] + s[m]) / 2.0


def classify_lab(l, a, b):
    best = None
    best_d = COLOR_MAXDIST2
    for name in COLOR_REFS:
        rl, ra, rb = COLOR_REFS[name]
        d = (l - rl) * (l - rl) + (a - ra) * (a - ra) + (b - rb) * (b - rb)
        if d < best_d:
            best_d = d
            best = name
    return best


def classify_at(img, x, y, side):
    half = side // 2
    x0 = x - half
    y0 = y - half
    if x0 < 0:
        x0 = 0
    if y0 < 0:
        y0 = 0
    if x0 + side > IMG_W:
        x0 = IMG_W - side
    if y0 + side > IMG_H:
        y0 = IMG_H - side
    if x0 < 0 or y0 < 0 or side < 2:
        return None
    s = img.get_statistics(roi=(x0, y0, side, side))
    return classify_lab(s.l_mean(), s.a_mean(), s.b_mean())


def find_target_blob(img):
    blobs = img.find_blobs(
        DISC_THRESHOLDS,
        pixels_threshold=DISC_MIN_PIXELS,
        area_threshold=DISC_MIN_PIXELS,
        merge=True,
        margin=10,
    )
    if not blobs:
        return None
    best = None
    for b in blobs:
        if best is None or b.pixels() > best.pixels():
            best = b
    w = best.w()
    h = best.h()
    if w < DISC_MIN_DIM or h < DISC_MIN_DIM:
        return None
    if w > DISC_MAX_DIM or h > DISC_MAX_DIM:
        return None
    return best


def read_zone_distance(cx, cy):
    try:
        depth, _dmin, _dmax = tof.read_depth(timeout=200)
    except Exception:
        return None
    zc = int(cx * TOF_W / IMG_W)
    zr = int(cy * TOF_H / IMG_H)
    if zc < 0:
        zc = 0
    if zc > TOF_W - 1:
        zc = TOF_W - 1
    if zr < 0:
        zr = 0
    if zr > TOF_H - 1:
        zr = TOF_H - 1
    vals = []
    for r in range(zr - 1, zr + 2):
        if r < 0 or r >= TOF_H:
            continue
        for c in range(zc - 1, zc + 2):
            if c < 0 or c >= TOF_W:
                continue
            v = depth[r * TOF_W + c]
            if v is not None and 10.0 < v < 4000.0:
                vals.append(float(v))
    if not vals:
        return None
    return median(vals)


def radius_from_distance(d):
    denom = d - CAL_D0
    if denom <= 1.0:
        return None
    r = CAL_K / denom
    if r < R_PX_MIN:
        r = R_PX_MIN
    if r > R_PX_MAX:
        r = R_PX_MAX
    return r


def read_rings(img, cx, cy, radius, draw=False):
    rings = []
    confs = []
    for i in range(len(RING_FRACS)):
        frac = RING_FRACS[i]
        angles = INNER_ANGLES if i == 0 else FULL_ANGLES
        roi_side = int(radius * ROI_FRAC)
        if roi_side < 3:
            roi_side = 3
        sample_r = radius * frac
        votes = {}
        for ang in angles:
            rad = math.radians(ang)
            sx = int(cx + sample_r * math.cos(rad))
            sy = int(cy + sample_r * math.sin(rad))
            col = classify_at(img, sx, sy, roi_side)
            if draw:
                img.draw_circle(sx, sy, 1, thickness=1)
            if col:
                votes[col] = votes.get(col, 0) + 1
        if votes:
            best = None
            bv = 0
            for k in votes:
                if votes[k] > bv:
                    bv = votes[k]
                    best = k
            conf = bv / len(angles)
            if bv >= MIN_VOTES:
                rings.append(best)
                confs.append(conf)
            else:
                rings.append(None)
                confs.append(conf)
        else:
            rings.append(None)
            confs.append(0.0)
    return rings, confs


def read_rings_best(img, cx, cy, r0, draw=False):
    best = None
    for sc in R_SCAN:
        rings, confs = read_rings(img, cx, cy, r0 * sc, draw=False)
        score = 0.0
        for c in confs:
            score += c
        if best is None or score > best[0]:
            best = (score, rings, confs, r0 * sc)
    if draw:
        read_rings(img, cx, cy, best[3], draw=True)
    return best[1], best[2], best[3]


def strict_valid(rings, confs):
    if len(rings) != 5:
        return False
    for i in range(5):
        if rings[i] is None:
            return False
        if confs[i] < MIN_CONF_STRICT:
            return False
    return True


def rings_sum(rings):
    total = 0
    for r in rings:
        total += VALUES.get(r, 0)
    return total


def victim_from_sum(total):
    if total == 2:
        return "HARMED"
    if total == 1:
        return "STABLE"
    if total == 0:
        return "UNHARMED"
    return None


def classify_letter(img):
    """Run the FOMO model on the live frame and return a victim type or None.
    NOT based on saved images - uses the deployed .tflite model.
    """
    if letter_model is None:
        return None
    try:
        out = letter_model.predict([img])
        grid = out[0]                 # (1, gh, gw, n_classes) probabilities
        sh = grid.shape
        gh = sh[1]
        gw = sh[2]
        nc = sh[3]
        cls_max = [0.0] * nc
        for y in range(gh):
            for x in range(gw):
                for c in range(nc):
                    v = float(grid[0, y, x, c])
                    if v > cls_max[c]:
                        cls_max[c] = v
        best_label = None
        best_v = 0.0
        for c in range(nc):
            name = LETTER_LABELS_ORDER[c] if c < len(LETTER_LABELS_ORDER) else None
            if name is None or name == LETTER_BG_LABEL:
                continue
            if cls_max[c] > best_v:
                best_v = cls_max[c]
                best_label = name
        if best_label is None or best_v < LETTER_CONF:
            return None
        return LETTER_TO_VICTIM.get(best_label, None)
    except Exception:
        return None

# =============================================================================
# CONFIRM-READ STATE
# =============================================================================

cr_type = None
cr_hits = 0
cr_lost = 0


def confirm_reset():
    global cr_type, cr_hits, cr_lost
    cr_type = None
    cr_hits = 0
    cr_lost = 0


def confirm_read(img):
    global cr_type, cr_hits, cr_lost
    blob = find_target_blob(img)
    letter = classify_letter(img)

    if blob is None and letter is None:
        cr_lost += 1
        if cr_lost >= 8:
            return ("none",)
        return None
    cr_lost = 0

    cand = None
    if blob is not None:
        cx = blob.cx()
        cy = blob.cy()
        aspect = blob.w() / max(1, blob.h())
        if ASPECT_MIN <= aspect <= ASPECT_MAX:
            d = read_zone_distance(cx, cy)
            if d is not None:
                r0 = radius_from_distance(d)
                if r0 is not None:
                    blob_r = (blob.w() + blob.h()) / 4.0
                    if abs(blob_r - r0) <= RADIUS_AGREE_FRAC * r0:
                        rings, confs, r_used = read_rings_best(img, cx, cy, r0)
                        if strict_valid(rings, confs):
                            cand = victim_from_sum(rings_sum(rings))
                            if SHOW_DEBUG:
                                img.draw_circle(int(cx), int(cy), int(r_used),
                                                thickness=2)

    if cand is None and letter is not None:
        cand = letter

    if cand is None:
        cr_hits = 0
        cr_type = None
        return None

    if cand == cr_type:
        cr_hits += 1
    else:
        cr_type = cand
        cr_hits = 1

    if cr_hits >= CONFIRM_MATCHES:
        return ("victim", cr_type)
    return None

# =============================================================================
# VISION DEBUG  (no UART)
# =============================================================================


def vision_debug(img):
    blob = find_target_blob(img)
    if blob is None:
        img.draw_string(2, 2, "no target  FPS=%.1f" % clock.fps(), scale=2)
        return
    cx = blob.cx()
    cy = blob.cy()
    aspect = blob.w() / max(1, blob.h())
    img.draw_rectangle(blob.rect(), thickness=1)
    img.draw_cross(cx, cy, size=8, thickness=2)

    d = read_zone_distance(cx, cy)
    if d is None:
        img.draw_string(2, 2, "no ToF", scale=2)
        return
    r0 = radius_from_distance(d)
    if r0 is None:
        img.draw_string(2, 2, "bad r0", scale=2)
        return

    rings, confs, r_used = read_rings_best(img, cx, cy, r0, draw=True)
    img.draw_circle(int(cx), int(cy), int(r_used), thickness=2)

    valid = strict_valid(rings, confs)
    total = rings_sum(rings)
    vt = victim_from_sum(total) if valid else None
    label = vt if vt else ("FAKE" if valid else "INVALID")

    img.draw_string(2, 2, "d=%.0f r=%.0f asp=%.2f" % (d, r_used, aspect), scale=2)
    img.draw_string(2, 22, "%s sum=%d %s" % (
        "/".join([(r[0] if r else "?") for r in rings]), total, label), scale=2)

    print("d=%.0f r=%.0f asp=%.2f rings=%s confs=%s sum=%d -> %s" % (
        d, r_used, aspect,
        str([(r if r else "?") for r in rings]),
        str(["%.2f" % c for c in confs]),
        total, label))

# =============================================================================
# MAIN STATE MACHINE
# =============================================================================

ST_SEARCH = 0
ST_TX_STOP = 1
ST_READ = 2
ST_TX_VICTIM = 3
ST_TX_GO = 4
ST_COOLDOWN = 5

state = ST_SEARCH
state_t0 = time.ticks_ms()
suspect_hits = 0
victim_pending = None


def goto(s):
    global state, state_t0
    state = s
    state_t0 = time.ticks_ms()


print("Ready. DEBUG_VISION=%s  MCU_PRESENT=%s" % (DEBUG_VISION, MCU_PRESENT))

while True:

    clock.tick()
    img = sensor.snapshot()
    now = time.ticks_ms()

    if CALIBRATE_COLORS:
        s = img.get_statistics(roi=(IMG_W // 2 - 8, IMG_H // 2 - 8, 16, 16))
        img.draw_rectangle(IMG_W // 2 - 8, IMG_H // 2 - 8, 16, 16, thickness=2)
        print("centre LAB:  L=%.1f A=%.1f B=%.1f" % (
            s.l_mean(), s.a_mean(), s.b_mean()))
        continue

    if DEBUG_VISION:
        vision_debug(img)
        continue

    link.heartbeat(now)
    link_status = link.service(now)

    if state == ST_SEARCH:
        blob = find_target_blob(img)
        if blob is not None:
            suspect_hits += 1
        else:
            suspect_hits = 0
        if suspect_hits >= SUSPECT_HITS:
            suspect_hits = 0
            confirm_reset()
            if SEND_STOP:
                link.start_msg(T_STOP, STOP_TIMEOUT_MS)
                goto(ST_TX_STOP)
                if SHOW_DEBUG:
                    print("SUSPECT -> STOP")
            else:
                goto(ST_READ)

    elif state == ST_TX_STOP:
        if link_status == "acked":
            goto(ST_READ)
            if SHOW_DEBUG:
                print("STOP acked -> READ")
        elif link_status == "timeout":
            link.start_msg(T_GO, GO_TIMEOUT_MS)
            goto(ST_TX_GO)
            if SHOW_DEBUG:
                print("STOP timeout -> GO (safety)")

    elif state == ST_READ:
        res = confirm_read(img)
        if res is not None and res[0] == "victim":
            victim_pending = res[1]
            link.start_msg(VICTIM_BYTE[victim_pending], VICTIM_TIMEOUT_MS)
            goto(ST_TX_VICTIM)
            if SHOW_DEBUG:
                print("VICTIM:", victim_pending)
        elif (res is not None and res[0] == "none") or \
             time.ticks_diff(now, state_t0) >= READ_TIMEOUT_MS:
            link.start_msg(T_GO, GO_TIMEOUT_MS)
            goto(ST_TX_GO)
            if SHOW_DEBUG:
                print("no victim -> GO")

    elif state == ST_TX_VICTIM:
        if link_status in ("acked", "timeout"):
            link.start_msg(T_GO, GO_TIMEOUT_MS)
            goto(ST_TX_GO)
            if SHOW_DEBUG:
                print("victim %s -> GO" % link_status)

    elif state == ST_TX_GO:
        if link_status in ("acked", "timeout"):
            victim_pending = None
            goto(ST_COOLDOWN)
            if SHOW_DEBUG:
                print("GO %s -> COOLDOWN" % link_status)

    elif state == ST_COOLDOWN:
        if time.ticks_diff(now, state_t0) >= COOLDOWN_MS:
            suspect_hits = 0
            confirm_reset()
            goto(ST_SEARCH)

    if SHOW_DEBUG:
        img.draw_string(2, 2, "ST=%d FPS=%.1f" % (state, clock.fps()), scale=1)