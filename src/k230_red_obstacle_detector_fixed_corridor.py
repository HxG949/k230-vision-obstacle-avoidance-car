import gc
import math
import os
import struct
import time

import image
from media.display import *
from media.media import *
from media.sensor import *

try:
    from machine import UART, FPIOA
except Exception:
    try:
        from machine import UART
    except Exception:
        UART = None
    FPIOA = None


# K230 / CanMV red obstacle detector debug template.
# Current stage: infer full red-bottle obstacles and test them against a
# two-line image-space corridor calibrated from black floor tapes.

# Keep the IDE preview path identical to the first known-good version.
# The previous ST7701 480x640 path can keep the algorithm running while the
# image stream fails to return to the IDE, which makes field tuning painful.
SENSOR_WIDTH = 320
SENSOR_HEIGHT = 180
DETECT_WIDTH = 320
DETECT_HEIGHT = 180
DETECT_FPS = 30
DETECT_CHN = CAM_CHN_ID_1

# Orientation diagnostic/fallback.
# Keep this at 0 unless a firmware API is confirmed to return a new image with
# swapped dimensions. The previous rotation_corr(90) call kept 640x480 output
# and caused striped display corruption on a 480x640 display buffer.
ROTATE_IMAGE_DEGREES = 0
PRINT_IMAGE_SHAPE_ONCE = True
PRINT_SENSOR_ORIENTATION_API_ONCE = True

# Red detection ROI, in image pixels: (x, y, w, h).
OBSTACLE_ROI = (0, 0, 320, 180)

# Collision corridor. In CALIBRATE_LINES mode, two black floor tapes are
# detected and fitted as x = k*y + b. FIXED_LINES uses the constants below.
CORRIDOR_MODE = "FIXED_LINES"  # CALIBRATE_LINES, FIXED_LINES
CORRIDOR_TOP_Y = 70
CORRIDOR_BOTTOM_Y = 179
FIXED_LEFT_LINE_K = -0.36
FIXED_LEFT_LINE_B = 144.0
FIXED_RIGHT_LINE_K = 0.36
FIXED_RIGHT_LINE_B = 176.0

# Black floor tape detection. CanMV thresholds are LAB:
# (L_min, L_max, A_min, A_max, B_min, B_max)
BLACK_THRESHOLDS = [
    (0, 35, -128, 127, -128, 127),
]
BLACK_LINE_ROI = (0, 0, 320, 180)
BLACK_MIN_PIXELS = 35
BLACK_MIN_AREA = 45
BLACK_MAX_AREA = 12000
BLACK_MIN_H = 18
BLACK_MIN_ASPECT_H_OVER_W = 1.15
BLACK_LINE_MERGE_MARGIN = 8

# CanMV/OpenMV find_blobs() uses LAB thresholds:
# (L_min, L_max, A_min, A_max, B_min, B_max)
RED_THRESHOLDS = [
    (20, 100, 20, 127, -10, 127),
    (10, 80, 35, 127, 0, 127),
]

# Red candidate filtering.
MIN_PIXELS = 35
MIN_AREA = 45
MAX_AREA = 12000
MIN_W = 3
MIN_H = 4
MIN_ASPECT = 0.15
MAX_ASPECT = 3.20
MIN_DENSITY = 0.12
MERGE_MARGIN = 30
MAX_CANDIDATES = 8

# Bottle candidate inference from red features.
# Red blobs are features, not obstacles by themselves. These parameters merge
# cap/label red boxes and expand them to approximate the whole bottle body.
BOTTLE_GROUP_MAX_CENTER_DX = 50
BOTTLE_GROUP_MAX_VERTICAL_GAP = 72
BOTTLE_PADDING_X = 10
BOTTLE_PADDING_TOP = 6
BOTTLE_PADDING_BOTTOM = 22
BOTTLE_MIN_W = 12
BOTTLE_MIN_H = 18
BOTTLE_SINGLE_LABEL_HEIGHT_SCALE = 1.80
BOTTLE_SINGLE_CAP_HEIGHT_SCALE = 4.00
BOTTLE_SINGLE_CAP_MAX_H = 36
MAX_BOTTLES = 5

# Bottom center is closer to the ground contact point for a forward/down camera.
POINT_MODE = "bottom_center"

# Avoidance motion in chassis right-handed coordinates:
# x+ forward, y+ left, z+ counterclockwise.
TASK_OBSTACLE_CROSSING = 1
TARGET_CHASSIS = 0
TARGET_GIMBAL = 1

FORWARD_SPEED = 0.22
FORWARD_TIME = 0.25
SHIFT_SPEED = 0.20
SHIFT_TIME = 0.25
DEFAULT_AVOID_SIDE = 1       # 1 -> shift left first, -1 -> shift right first.
AVOID_LEFT_ONLY = True       # Current obstacle strategy: near obstacle -> shift left.
MAX_XY_SPEED = 0.60

# TI <-> K230 stage protocol.
# Packet format: [stage_id][command].
# Stage 1:
#   0x12 0xFF -> stage 1 starts; TI runs the open-loop route.
#   0x12 0x1B -> stage 1 open-loop route done; K230 starts vision feedback.
# Stage 2:
#   0x13 0xFF -> stage 2 starts; K230 stops vision feedback.
TI_STAGE1_ID = 0x12
TI_STAGE2_ID = 0x13
TI_CMD_START = 0xFF
TI_CMD_STAGE1_OPEN_LOOP_DONE = 0x1B
K230_FLAG_BLOCKED = 0    # K230 sends: 0x12 0x00, keep shifting left.
K230_FLAG_CLEAR = 1      # K230 sends: 0x12 0x01, corridor clear.
SEND_FLAG_EVERY_N_FRAMES = 2

STATE_WAIT_STAGE1 = 0
STATE_VISION_AVOID = 1
STATE_STAGE2_STOPPED = 2

TI_EVENT_NONE = 0
TI_EVENT_STAGE1_START = 1
TI_EVENT_STAGE1_OPEN_LOOP_DONE = 2
TI_EVENT_STAGE2_START = 3

# Optional display/debug output.
ENABLE_DISPLAY = True
DRAW_DEBUG = True
PRINT_EVERY_N_FRAMES = 6
DISPLAY_EVERY_N_FRAMES = 1

# Display pipeline diagnostic.
# True: show the raw camera frame immediately after snapshot, before blob
# detection and debug drawing. This isolates whether later allocations consume
# the display buffer pool on CanMV.
DISPLAY_BEFORE_ANALYSIS = True
DEBUG_VIEW_MODE = "BOTTLE"  # RAW, BOTTLE, CORRIDOR, LINES
DRAW_RED_FEATURES_IN_BOTTLE_VIEW = False

# IDE virtual display matches the first version that returned smooth video.
DISPLAY_QUALITY = 35

# Bench mode before the camera is mounted on the car.
# True: only detect/draw/print red obstacle state, never send motion commands.
# False: use TI strict-wait communication and send chassis commands.
CAMERA_ONLY_TEST = False

# UART settings. Fill tx_pin/rx_pin according to your K230 carrier board.
UART_ENABLED = True
UART_ID = 3
UART_BAUDRATE = 115200
UART_TX_PIN = 32
UART_RX_PIN = 33


class ObstacleCandidate:
    def __init__(self, blob, score, point_img):
        self.blob = blob
        self.score = score
        self.point_img = point_img


class BottleCandidate:
    def __init__(self, rect, red_parts, score):
        self.rect = rect
        self.red_parts = red_parts
        self.score = score
        x, y, w, h = rect
        self.point_img = (x + w // 2, y + h)


class CorridorLine:
    def __init__(self, k, b, valid=False):
        self.k = k
        self.b = b
        self.valid = valid

    def x_at(self, y):
        return self.k * y + self.b


class Corridor:
    def __init__(self, left, right, top_y, bottom_y, source="none"):
        self.left = left
        self.right = right
        self.top_y = top_y
        self.bottom_y = bottom_y
        self.source = source

    def valid(self):
        return self.left.valid and self.right.valid

    def poly(self, img_w=None):
        lt = self.left.x_at(self.top_y)
        lb = self.left.x_at(self.bottom_y)
        rt = self.right.x_at(self.top_y)
        rb = self.right.x_at(self.bottom_y)
        if img_w is not None:
            lt = max(0, min(int(lt), img_w - 1))
            lb = max(0, min(int(lb), img_w - 1))
            rt = max(0, min(int(rt), img_w - 1))
            rb = max(0, min(int(rb), img_w - 1))
        return [
            (int(lt), self.top_y),
            (int(rt), self.top_y),
            (int(rb), self.bottom_y),
            (int(lb), self.bottom_y),
        ]


class RunState:
    def __init__(self):
        self.phase = STATE_VISION_AVOID if CAMERA_ONLY_TEST else STATE_WAIT_STAGE1
        self.rx_buf = bytearray()
        self.last_corridor = get_fixed_corridor()


def setup_sensor():
    sensor = Sensor(width=SENSOR_WIDTH, height=SENSOR_HEIGHT, fps=DETECT_FPS)
    sensor.reset()
    sensor.set_framesize(width=SENSOR_WIDTH, height=SENSOR_HEIGHT, chn=DETECT_CHN)
    sensor.set_pixformat(Sensor.RGB565, chn=DETECT_CHN)
    sensor.run()
    time.sleep_ms(200)
    return sensor


def print_sensor_orientation_api(sensor):
    names = [
        "set_hmirror",
        "set_vflip",
        "set_transpose",
        "set_rotation",
        "set_auto_rotation",
        "set_pixformat",
        "set_framesize",
    ]
    found = []
    for name in names:
        if hasattr(sensor, name):
            found.append(name)
    print("SENSOR_ORIENT_API,%s" % ("|".join(found) if found else "none"))


def setup_uart():
    if not UART_ENABLED:
        return None
    if UART is None:
        raise RuntimeError("UART is not available on this firmware")

    uart_id = getattr(UART, "UART%d" % UART_ID, UART_ID)

    if FPIOA is not None and (UART_TX_PIN is not None or UART_RX_PIN is not None):
        fpioa = FPIOA()
        tx_func = getattr(FPIOA, "UART%d_TXD" % UART_ID, None)
        rx_func = getattr(FPIOA, "UART%d_RXD" % UART_ID, None)
        if UART_TX_PIN is not None and tx_func is not None:
            fpioa.set_function(UART_TX_PIN, tx_func)
        if UART_RX_PIN is not None and rx_func is not None:
            fpioa.set_function(UART_RX_PIN, rx_func)

    return UART(uart_id, UART_BAUDRATE)


def capture_image(sensor):
    img = sensor.snapshot(chn=DETECT_CHN)
    if ROTATE_IMAGE_DEGREES:
        return rotate_image(img, ROTATE_IMAGE_DEGREES)
    return img


def rotate_image(img, degrees):
    """Best-effort rotation fallback.

    If this firmware lacks rotation_corr(), leave the frame unchanged and print
    a clear diagnostic instead of crashing during field debugging.
    """
    if hasattr(img, "rotation_corr"):
        try:
            return img.rotation_corr(z_rotation=degrees)
        except Exception as e:
            print("ROTATE_FAIL,%s" % e)
            return img
    print("ROTATE_UNSUPPORTED,check camera mount or firmware image API")
    return img


def get_obstacle_roi(img):
    x, y, w, h = OBSTACLE_ROI
    x = max(0, min(x, img.width() - 1))
    y = max(0, min(y, img.height() - 1))
    w = max(1, min(w, img.width() - x))
    h = max(1, min(h, img.height() - y))
    return (x, y, w, h)


def get_black_line_roi(img):
    x, y, w, h = BLACK_LINE_ROI
    x = max(0, min(x, img.width() - 1))
    y = max(0, min(y, img.height() - 1))
    w = max(1, min(w, img.width() - x))
    h = max(1, min(h, img.height() - y))
    return (x, y, w, h)


def get_fixed_corridor():
    return Corridor(
        CorridorLine(FIXED_LEFT_LINE_K, FIXED_LEFT_LINE_B, True),
        CorridorLine(FIXED_RIGHT_LINE_K, FIXED_RIGHT_LINE_B, True),
        CORRIDOR_TOP_Y,
        CORRIDOR_BOTTOM_Y,
        "FIXED",
    )


def detect_black_line_blobs(img):
    return img.find_blobs(
        BLACK_THRESHOLDS,
        roi=get_black_line_roi(img),
        x_stride=2,
        y_stride=2,
        pixels_threshold=BLACK_MIN_PIXELS,
        area_threshold=BLACK_MIN_AREA,
        merge=True,
        margin=BLACK_LINE_MERGE_MARGIN,
    )


def is_line_blob(blob):
    if blob.area() > BLACK_MAX_AREA or blob.h() < BLACK_MIN_H:
        return False
    ratio = float(blob.h()) / max(float(blob.w()), 1.0)
    return ratio >= BLACK_MIN_ASPECT_H_OVER_W


def fit_x_from_y(points):
    n = len(points)
    if n < 2:
        return CorridorLine(0.0, 0.0, False)
    sum_y = 0.0
    sum_x = 0.0
    sum_yy = 0.0
    sum_yx = 0.0
    for x, y in points:
        fy = float(y)
        fx = float(x)
        sum_y += fy
        sum_x += fx
        sum_yy += fy * fy
        sum_yx += fy * fx
    denom = n * sum_yy - sum_y * sum_y
    if abs(denom) < 1e-6:
        return CorridorLine(0.0, sum_x / max(n, 1), True)
    k = (n * sum_yx - sum_y * sum_x) / denom
    b = (sum_x - k * sum_y) / n
    return CorridorLine(k, b, True)


def corridor_from_black_lines(img, last_corridor):
    blobs = detect_black_line_blobs(img)
    line_blobs = []
    for blob in blobs:
        if is_line_blob(blob):
            line_blobs.append(blob)

    if len(line_blobs) < 2:
        return last_corridor, line_blobs

    line_blobs.sort(key=lambda b: b.cx())
    left_group = line_blobs[: max(1, len(line_blobs) // 2)]
    right_group = line_blobs[max(1, len(line_blobs) // 2):]
    if not right_group:
        return last_corridor, line_blobs

    left_points = line_points_from_blobs(left_group)
    right_points = line_points_from_blobs(right_group)
    left = fit_x_from_y(left_points)
    right = fit_x_from_y(right_points)
    corridor = Corridor(left, right, CORRIDOR_TOP_Y, CORRIDOR_BOTTOM_Y, "BLACK")
    if not corridor.valid():
        return last_corridor, line_blobs
    if corridor.left.x_at(CORRIDOR_BOTTOM_Y) > corridor.right.x_at(CORRIDOR_BOTTOM_Y):
        corridor.left, corridor.right = corridor.right, corridor.left
    return corridor, line_blobs


def line_points_from_blobs(blobs):
    points = []
    for blob in blobs:
        x_mid = blob.x() + blob.w() // 2
        points.append((x_mid, blob.y()))
        points.append((x_mid, blob.y() + blob.h()))
        points.append((blob.cx(), blob.cy()))
    return points


def get_corridor(img, last_corridor):
    if CORRIDOR_MODE == "CALIBRATE_LINES":
        return corridor_from_black_lines(img, last_corridor)
    return get_fixed_corridor(), []


def detect_red_regions(img, roi):
    return img.find_blobs(
        RED_THRESHOLDS,
        roi=roi,
        x_stride=2,
        y_stride=2,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=True,
        margin=MERGE_MARGIN,
    )


def score_blob(blob, roi):
    w = blob.w()
    h = blob.h()
    if w < MIN_W or h < MIN_H:
        return -1.0
    area = blob.area()
    if area < MIN_AREA or area > MAX_AREA:
        return -1.0
    aspect = float(w) / max(float(h), 1.0)
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return -1.0
    density = blob.density()
    if density < MIN_DENSITY:
        return -1.0

    rx, ry, rw, rh = roi
    area_score = min(float(blob.pixels()) / 900.0, 1.0)
    density_score = min(max((density - MIN_DENSITY) / 0.50, 0.0), 1.0)
    nx = abs(blob.cx() - (rx + rw * 0.5)) / max(rw * 0.5, 1.0)
    ny = abs(blob.cy() - (ry + rh * 0.58)) / max(rh * 0.58, 1.0)
    position_score = max(0.0, 1.0 - (0.45 * nx + 0.35 * ny))
    aspect_score = 1.0 - min(abs(math.log(max(aspect, 0.01))), 1.4) / 1.4
    return (
        0.35 * area_score
        + 0.25 * density_score
        + 0.25 * position_score
        + 0.15 * aspect_score
    )


def filter_candidates(blobs, roi):
    candidates = []
    for blob in blobs:
        score = score_blob(blob, roi)
        if score < 0.15:
            continue
        candidates.append(ObstacleCandidate(blob, score, get_representative_point(blob)))

    candidates.sort(key=lambda c: c.score, reverse=True)
    candidates = suppress_overlaps(candidates)
    return candidates[:MAX_CANDIDATES]


def build_bottle_candidates(red_candidates, img):
    """Infer whole-bottle boxes from red cap/label feature boxes."""
    groups = group_red_features(red_candidates)
    bottles = []
    for group in groups:
        rect = infer_bottle_rect(group, img.width(), img.height())
        if rect[2] < BOTTLE_MIN_W or rect[3] < BOTTLE_MIN_H:
            continue
        score = 0.0
        for cand in group:
            score += cand.score
        score /= max(len(group), 1)
        bottles.append(BottleCandidate(rect, group, score))

    bottles.sort(key=lambda b: b.score, reverse=True)
    bottles = suppress_bottle_overlaps(bottles)
    return bottles[:MAX_BOTTLES]


def group_red_features(red_candidates):
    ordered = sorted(red_candidates, key=lambda c: c.blob.cx())
    groups = []
    used = [False] * len(ordered)

    for i, cand in enumerate(ordered):
        if used[i]:
            continue
        group = [cand]
        used[i] = True
        changed = True
        while changed:
            changed = False
            gx = group_center_x(group)
            gy1, gy2 = group_y_range(group)
            for j, other in enumerate(ordered):
                if used[j]:
                    continue
                dx = abs(other.blob.cx() - gx)
                oy1 = other.blob.y()
                oy2 = other.blob.y() + other.blob.h()
                vertical_gap = max(0, max(gy1, oy1) - min(gy2, oy2))
                if dx <= BOTTLE_GROUP_MAX_CENTER_DX and vertical_gap <= BOTTLE_GROUP_MAX_VERTICAL_GAP:
                    group.append(other)
                    used[j] = True
                    changed = True
                    gy1, gy2 = group_y_range(group)
        groups.append(group)
    return groups


def group_center_x(group):
    total_weight = 0
    weighted_x = 0
    for cand in group:
        weight = max(cand.blob.pixels(), 1)
        weighted_x += cand.blob.cx() * weight
        total_weight += weight
    return weighted_x / max(total_weight, 1)


def group_y_range(group):
    y1 = min(c.blob.y() for c in group)
    y2 = max(c.blob.y() + c.blob.h() for c in group)
    return y1, y2


def infer_bottle_rect(group, img_w, img_h):
    min_x = min(c.blob.x() for c in group)
    min_y = min(c.blob.y() for c in group)
    max_x = max(c.blob.x() + c.blob.w() for c in group)
    max_y = max(c.blob.y() + c.blob.h() for c in group)

    if len(group) == 1:
        blob = group[0].blob
        if blob.h() <= BOTTLE_SINGLE_CAP_MAX_H:
            extra_bottom = int(blob.h() * BOTTLE_SINGLE_CAP_HEIGHT_SCALE)
            extra_top = BOTTLE_PADDING_TOP
        else:
            extra_bottom = int(blob.h() * (BOTTLE_SINGLE_LABEL_HEIGHT_SCALE - 1.0))
            extra_top = int(blob.h() * 0.45)
    else:
        extra_bottom = BOTTLE_PADDING_BOTTOM
        extra_top = BOTTLE_PADDING_TOP

    x = min_x - BOTTLE_PADDING_X
    y = min_y - extra_top
    w = (max_x - min_x) + 2 * BOTTLE_PADDING_X
    h = (max_y - min_y) + extra_top + extra_bottom
    return clamp_rect((x, y, w, h), img_w, img_h)


def clamp_rect(rect, img_w, img_h):
    x, y, w, h = rect
    x = max(0, min(int(x), img_w - 1))
    y = max(0, min(int(y), img_h - 1))
    w = max(1, min(int(w), img_w - x))
    h = max(1, min(int(h), img_h - y))
    return (x, y, w, h)


def suppress_bottle_overlaps(bottles):
    kept = []
    for bottle in bottles:
        overlapped = False
        for old in kept:
            if rect_iou(bottle.rect, old.rect) > 0.45:
                overlapped = True
                break
        if not overlapped:
            kept.append(bottle)
    return kept


def suppress_overlaps(candidates):
    kept = []
    for cand in candidates:
        overlapped = False
        for old in kept:
            if rect_iou(cand.blob.rect(), old.blob.rect()) > 0.35:
                overlapped = True
                break
        if not overlapped:
            kept.append(cand)
    return kept


def rect_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax2 = ax + aw
    ay2 = ay + ah
    bx2 = bx + bw
    by2 = by + bh
    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    if union <= 0:
        return 0.0
    return float(inter) / float(union)


def get_representative_point(blob):
    if POINT_MODE == "center":
        return (blob.cx(), blob.cy())
    return (blob.x() + blob.w() // 2, blob.y() + blob.h())


def point_in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        crosses = ((yi > y) != (yj > y))
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / float(yj - yi) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def rect_contains_point(rect, point):
    x, y, w, h = rect
    px, py = point
    return px >= x and px <= x + w and py >= y and py <= y + h


def rect_to_poly(rect):
    x, y, w, h = rect
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]


def ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def rect_intersects_poly(rect, poly):
    rect_pts = rect_to_poly(rect)

    for px, py in rect_pts:
        if point_in_poly(px, py, poly):
            return True
    for p in poly:
        if rect_contains_point(rect, p):
            return True

    rect_edges = [
        (rect_pts[0], rect_pts[1]),
        (rect_pts[1], rect_pts[2]),
        (rect_pts[2], rect_pts[3]),
        (rect_pts[3], rect_pts[0]),
    ]
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        for c, d in rect_edges:
            if segments_intersect(a, b, c, d):
                return True
    return False


def rect_inside_poly(rect, poly):
    for px, py in rect_to_poly(rect):
        if not point_in_poly(px, py, poly):
            return False
    return True


def rect_intersects_rect(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    return not (
        ax + aw < bx
        or bx + bw < ax
        or ay + ah < by
        or by + bh < ay
    )


def is_bottle_in_corridor(bottle, corridor, img_w):
    corridor_poly = corridor.poly(img_w)
    return rect_inside_poly(bottle.rect, corridor_poly) or rect_intersects_poly(
        bottle.rect, corridor_poly
    )


def analyze_corridor(bottles, corridor, img_w):
    inside = []
    left_weight = 0
    right_weight = 0
    max_bottom_y = 0
    nearest = None
    center_x = (
        corridor.left.x_at(corridor.bottom_y)
        + corridor.right.x_at(corridor.bottom_y)
    ) * 0.5
    for bottle in bottles:
        if not is_bottle_in_corridor(bottle, corridor, img_w):
            continue
        inside.append(bottle)
        x, y, w, h = bottle.rect
        bottom_y = y + h
        if bottom_y > max_bottom_y:
            max_bottom_y = bottom_y
            nearest = bottle
        cx = x + w * 0.5
        weight = max(w * h, 1)
        if cx < center_x:
            left_weight += weight
        else:
            right_weight += weight
    return inside, left_weight, right_weight, max_bottom_y, nearest


def choose_avoid_side(left_weight, right_weight):
    """Return 1 for left shift, -1 for right shift."""
    if AVOID_LEFT_ONLY:
        return 1
    if left_weight > right_weight * 1.15:
        return -1
    if right_weight > left_weight * 1.15:
        return 1
    return DEFAULT_AVOID_SIDE


def clamp_xy_speed(speed):
    if speed > MAX_XY_SPEED:
        return MAX_XY_SPEED
    if speed < -MAX_XY_SPEED:
        return -MAX_XY_SPEED
    return speed


def make_chassis_command(task_id, x_speed, x_time, y_speed, y_time,
                         z_speed, z_time):
    x_speed = clamp_xy_speed(x_speed)
    y_speed = clamp_xy_speed(y_speed)
    return struct.pack(
        "<BBffffff",
        task_id,
        TARGET_CHASSIS,
        float(x_speed),
        float(x_time),
        float(y_speed),
        float(y_time),
        float(z_speed),
        float(z_time),
    )


def make_gimbal_command(task_id, pitch_dir, pitch_rpm, pitch_pulse,
                        yaw_dir, yaw_rpm, yaw_pulse):
    return struct.pack(
        "<BBBHIBHI",
        task_id,
        TARGET_GIMBAL,
        pitch_dir,
        pitch_rpm,
        pitch_pulse,
        yaw_dir,
        yaw_rpm,
        yaw_pulse,
    )


def make_forward_command(task_id):
    return make_chassis_command(
        task_id, FORWARD_SPEED, FORWARD_TIME, 0.0, 0.0, 0.0, 0.0
    )


def make_forward_speed_command(task_id, speed):
    return make_chassis_command(
        task_id, speed, FORWARD_TIME, 0.0, 0.0, 0.0, 0.0
    )


def make_shift_command(task_id, side):
    return make_chassis_command(
        task_id, 0.0, 0.0, side * SHIFT_SPEED, SHIFT_TIME, 0.0, 0.0
    )


def read_ti_stage_packet(uart, run_state):
    if uart is None:
        return TI_EVENT_NONE
    data = uart.read()
    if not data:
        return TI_EVENT_NONE
    for value in data:
        run_state.rx_buf.append(value)
    if len(run_state.rx_buf) > 12:
        run_state.rx_buf = trim_rx_to_tail(run_state.rx_buf, 12)

    idx = 0
    found = TI_EVENT_NONE
    while idx + 1 < len(run_state.rx_buf):
        stage_id = run_state.rx_buf[idx]
        command = run_state.rx_buf[idx + 1]
        if stage_id == TI_STAGE1_ID and command == TI_CMD_START:
            found = TI_EVENT_STAGE1_START
            run_state.rx_buf = consume_rx(run_state.rx_buf, idx + 2)
            idx = 0
            continue
        if stage_id == TI_STAGE1_ID and command == TI_CMD_STAGE1_OPEN_LOOP_DONE:
            found = TI_EVENT_STAGE1_OPEN_LOOP_DONE
            run_state.rx_buf = consume_rx(run_state.rx_buf, idx + 2)
            idx = 0
            continue
        if stage_id == TI_STAGE2_ID and command == TI_CMD_START:
            found = TI_EVENT_STAGE2_START
            run_state.rx_buf = consume_rx(run_state.rx_buf, idx + 2)
            idx = 0
            continue
        idx += 1
    return found


def consume_rx(rx_buf, count):
    next_buf = bytearray()
    for i in range(count, len(rx_buf)):
        next_buf.append(rx_buf[i])
    return next_buf


def trim_rx_to_tail(rx_buf, keep):
    if len(rx_buf) <= keep:
        return rx_buf
    next_buf = bytearray()
    start = len(rx_buf) - keep
    for i in range(start, len(rx_buf)):
        next_buf.append(rx_buf[i])
    return next_buf


def send_command(uart, packet):
    if uart is not None:
        uart.write(packet)


def send_clear_flag(uart, clear):
    flag = K230_FLAG_CLEAR if clear else K230_FLAG_BLOCKED
    if uart is not None:
        uart.write(bytes([TI_STAGE1_ID, flag]))
    return flag


LAST_PREVIEW_STATE = None


def draw_line(img, p1, p2, color, thickness=2):
    img.draw_line(p1[0], p1[1], p2[0], p2[1], color=color, thickness=thickness)


def draw_poly(img, poly, color, thickness=2):
    for i in range(len(poly)):
        draw_line(img, poly[i], poly[(i + 1) % len(poly)], color, thickness)


def draw_bottle_rects(img, bottle_rects, inside_flags):
    for i, rect in enumerate(bottle_rects):
        inside = i < len(inside_flags) and inside_flags[i]
        color = (0, 0, 255) if inside else (0, 180, 255)
        img.draw_rectangle(rect, color=color, thickness=2)


def make_preview_state(corridor, line_blobs, red_candidates, bottles, inside, action):
    inside_rects = []
    for bottle in inside:
        inside_rects.append(bottle.rect)

    line_rects = []
    for blob in line_blobs:
        line_rects.append(blob.rect())

    red_items = []
    for cand in red_candidates:
        red_items.append((cand.blob.rect(), cand.point_img))

    bottle_rects = []
    inside_flags = []
    for bottle in bottles:
        bottle_rects.append(bottle.rect)
        inside_flags.append(bottle.rect in inside_rects)

    return {
        "corridor_poly": corridor.poly(DETECT_WIDTH),
        "corridor_source": corridor.source,
        "left_k": corridor.left.k,
        "left_b": corridor.left.b,
        "right_k": corridor.right.k,
        "right_b": corridor.right.b,
        "line_rects": line_rects,
        "red_items": red_items,
        "bottle_rects": bottle_rects,
        "inside_flags": inside_flags,
        "action": action,
    }


def poly_to_log(poly):
    parts = []
    for x, y in poly:
        parts.append("%d:%d" % (x, y))
    return "|".join(parts)


def rects_to_log(items):
    parts = []
    for item in items:
        x, y, w, h = item.rect
        parts.append("%d:%d:%d:%d" % (x, y, w, h))
    return "|".join(parts) if parts else "none"


def corridor_to_log(corridor):
    return "%s,L,%.3f,%.1f,R,%.3f,%.1f" % (
        corridor.source,
        corridor.left.k,
        corridor.left.b,
        corridor.right.k,
        corridor.right.b,
    )


def draw_debug(img, roi, corridor_poly, red_candidates, bottles, inside, action, fps):
    rx, ry, rw, rh = roi
    img.draw_rectangle((rx, ry, rw, rh), color=(0, 160, 255), thickness=1)
    draw_poly(img, corridor_poly, (255, 255, 0) if inside else (0, 255, 0), 2)
    img.draw_string_advanced(2, 2, 20, "FPS %.1f" % fps, color=(255, 255, 0))
    img.draw_string_advanced(2, 24, 18, action, color=(255, 255, 255))

    for cand in red_candidates:
        blob = cand.blob
        img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=1)
        img.draw_cross(cand.point_img[0], cand.point_img[1],
                       color=(255, 255, 0), size=6, thickness=2)

    for bottle in bottles:
        color = (255, 0, 0) if bottle in inside else (0, 128, 255)
        img.draw_rectangle(bottle.rect, color=color, thickness=2)


def draw_bottle_preview(img, preview_state):
    if not preview_state:
        return

    draw_bottle_rects(
        img,
        preview_state["bottle_rects"],
        preview_state["inside_flags"],
    )

    if not DRAW_RED_FEATURES_IN_BOTTLE_VIEW:
        return

    for item in preview_state["red_items"]:
        rect, point = item
        img.draw_rectangle(rect, color=(0, 255, 0), thickness=1)
        img.draw_cross(point[0], point[1], color=(0, 0, 255), size=5, thickness=1)


def draw_corridor_preview(img, roi, corridor_poly, preview_state):
    rx, ry, rw, rh = roi
    img.draw_rectangle((rx, ry, rw, rh), color=(0, 160, 255), thickness=1)
    color = (0, 255, 0)
    if preview_state and preview_state["action"].find("IN_CORRIDOR") >= 0:
        color = (0, 220, 255)
    draw_poly(img, corridor_poly, color, 2)


def draw_line_calibration_preview(img, preview_state):
    if not preview_state:
        return
    draw_poly(img, preview_state["corridor_poly"], (0, 255, 0), 2)
    for rect in preview_state["line_rects"]:
        img.draw_rectangle(rect, color=(0, 180, 255), thickness=1)


def draw_selected_preview(img, roi, corridor_poly, preview_state):
    if DEBUG_VIEW_MODE == "BOTTLE":
        draw_bottle_preview(img, preview_state)
    elif DEBUG_VIEW_MODE == "CORRIDOR":
        draw_corridor_preview(img, roi, corridor_poly, preview_state)
    elif DEBUG_VIEW_MODE == "LINES":
        draw_line_calibration_preview(img, preview_state)


def run_once(sensor, uart, run_state, frame_id=0):
    global LAST_PREVIEW_STATE

    img = capture_image(sensor)
    roi = get_obstacle_roi(img)
    corridor = run_state.last_corridor
    line_blobs = []
    corridor_poly = corridor.poly(img.width())
    if DISPLAY_BEFORE_ANALYSIS and ENABLE_DISPLAY and frame_id % DISPLAY_EVERY_N_FRAMES == 0:
        draw_selected_preview(img, roi, corridor_poly, LAST_PREVIEW_STATE)
        Display.show_image(img)

    stage_packet = read_ti_stage_packet(uart, run_state)
    if stage_packet == TI_EVENT_STAGE1_START:
        run_state.phase = STATE_WAIT_STAGE1
        print("TI_STAGE,1_START")
    elif stage_packet == TI_EVENT_STAGE1_OPEN_LOOP_DONE:
        run_state.phase = STATE_VISION_AVOID
        print("TI_STAGE,1_OPEN_LOOP_DONE")
    elif stage_packet == TI_EVENT_STAGE2_START:
        run_state.phase = STATE_STAGE2_STOPPED
        print("TI_STAGE,2_START")

    if run_state.phase != STATE_VISION_AVOID:
        action = "WAIT_STAGE1" if run_state.phase == STATE_WAIT_STAGE1 else "STAGE2_STOP"
        if frame_id % PRINT_EVERY_N_FRAMES == 0:
            print("MODE,%s,%s" % ("CAM_TEST" if CAMERA_ONLY_TEST else "CAR", action))
        return img, roi, corridor_poly, [], [], [], action, run_state

    corridor, line_blobs = get_corridor(img, run_state.last_corridor)
    run_state.last_corridor = corridor
    corridor_poly = corridor.poly(img.width())

    blobs = detect_red_regions(img, roi)
    red_candidates = filter_candidates(blobs, roi)
    bottles = build_bottle_candidates(red_candidates, img)
    inside, left_weight, right_weight, max_bottom_y, nearest = analyze_corridor(
        bottles, corridor, img.width()
    )

    action = "WAIT"
    clear = len(inside) == 0
    if clear:
        action = "OUT_CORRIDOR CLEAR"
    else:
        action = "IN_CORRIDOR BLOCKED"

    sent_flag = None
    if frame_id % SEND_FLAG_EVERY_N_FRAMES == 0:
        sent_flag = send_clear_flag(uart, clear)

    if frame_id % PRINT_EVERY_N_FRAMES == 0:
        print(
            "MODE,%s,PHASE,%d,%s,FLAG,%s,RED,%d,BOTTLE,%d,IN_CORRIDOR,%d,L,%d,R,%d,BOTTOM_Y,%d,CORRIDOR,%s,HIT,%s"
            % (
                "CAM_TEST" if CAMERA_ONLY_TEST else "CAR",
                run_state.phase,
                action,
                "none" if sent_flag is None else str(sent_flag),
                len(red_candidates),
                len(bottles),
                len(inside),
                left_weight,
                right_weight,
                max_bottom_y,
                corridor_to_log(corridor),
                rects_to_log(inside),
            )
        )

    LAST_PREVIEW_STATE = make_preview_state(
        corridor, line_blobs, red_candidates, bottles, inside, action
    )

    return img, roi, corridor_poly, red_candidates, bottles, inside, action, run_state


def main():
    sensor = None
    uart = None
    display_inited = False
    media_inited = False
    clock = time.clock()
    frame_id = 0
    run_state = RunState()
    printed_shape = False

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
        if ENABLE_DISPLAY:
            Display.init(Display.VIRT, width=DETECT_WIDTH, height=DETECT_HEIGHT,
                         fps=30, to_ide=True, quality=DISPLAY_QUALITY)
            display_inited = True

        MediaManager.init()
        media_inited = True

        sensor = setup_sensor()
        if PRINT_SENSOR_ORIENTATION_API_ONCE:
            print_sensor_orientation_api(sensor)
        uart = setup_uart()
        if CAMERA_ONLY_TEST:
            print("K230 red obstacle camera-only test started")
        else:
            print("K230 red obstacle avoidance started")

        while True:
            clock.tick()
            os.exitpoint()

            img, roi, corridor_poly, red_candidates, bottles, inside, action, run_state = run_once(
                sensor, uart, run_state, frame_id
            )
            fps = clock.fps()

            if PRINT_IMAGE_SHAPE_ONCE and not printed_shape:
                print(
                    "IMAGE_SHAPE,w,%d,h,%d,rotate_deg,%d,display,%d,%d"
                    % (
                        img.width(),
                        img.height(),
                        ROTATE_IMAGE_DEGREES,
                        DETECT_WIDTH,
                        DETECT_HEIGHT,
                    )
                )
                printed_shape = True

            if (
                ENABLE_DISPLAY
                and DRAW_DEBUG
                and not DISPLAY_BEFORE_ANALYSIS
                and frame_id % DISPLAY_EVERY_N_FRAMES == 0
            ):
                draw_debug(img, roi, corridor_poly, red_candidates, bottles, inside, action, fps)
                Display.show_image(img)

            frame_id += 1
            gc.collect()

    except KeyboardInterrupt:
        print("stopped")
    except BaseException as e:
        print("Exception: %s" % e)
    finally:
        if sensor is not None:
            sensor.stop()
        if display_inited:
            Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        if media_inited:
            MediaManager.deinit()


if __name__ == "__main__":
    main()
