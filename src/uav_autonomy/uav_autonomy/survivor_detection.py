"""Survivor candidate detection in the downward camera image (no ROS).

SIMULATION BASELINE. The survivor in the Gazebo world is a manikin with
orange clothing, a skin-coloured head and blue legs. This detector looks for
that appearance with colour and shape cues. It is NOT a general person
detector and makes no claim about real imagery.

Per frame
    1. Clothing mask: saturated orange (HSV). Shadowed clothing is dark but
       stays saturated, so the value threshold is low and the saturation
       threshold high (the brown and tan debris boxes are less saturated).
    2. Connected clothing regions; regions whose boxes nearly touch are
       merged (an occluder can cut a body in two).
    3. For each region, cues that separate a person-like manikin from an
       orange object:
         head   a compact skin-coloured blob touching the clothing region,
                of plausible size relative to it
         shape  the clothing region is elongated AND does not fill its
                rectangle (arms spread from a torso; a box or a plank does
                not qualify)
         legs   a blue blob touching the clothing region, of plausible size
       A cue must be attached to the clothing: objects that are merely
       close to an orange object do not count.
    4. confidence = 0.30 (clothing) + 0.35 head + 0.20 shape + 0.15 legs
         >= strong_confidence  "strong": person-like cue set
         >= weak_confidence    "weak": an orange object of plausible size
                               (possibly a partly hidden survivor)
       Nothing here says "survivor": that needs temporal stability and the
       VLM (survivor_confirmation.py).

Bounding boxes and centres are in pixels of the full-resolution image even
when the masks are computed on a reduced copy.
"""

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np

SOURCE = 'hsv_shape_baseline'


@dataclass(frozen=True)
class DetectorParams:
    """Detector settings (docs/survivor_perception.md)."""

    scale: float = 0.5                    # masks computed at this image scale
    # OpenCV HSV: H 0..179, S and V 0..255
    clothing_h: Tuple[int, int] = (5, 22)
    clothing_s_min: int = 170
    clothing_v_min: int = 45
    skin_h: Tuple[int, int] = (8, 28)
    skin_s: Tuple[int, int] = (35, 125)
    skin_v_min: int = 70
    legs_h: Tuple[int, int] = (100, 128)
    legs_s_min: int = 120
    legs_v_min: int = 30
    min_area_px: float = 60.0             # clothing area, full-resolution px
    merge_gap_fraction: float = 0.35      # of the smaller box's larger side
    search_fraction: float = 0.45         # box growth when looking for cues
    head_area_ratio: Tuple[float, float] = (0.06, 2.5)   # skin / clothing
    head_min_fill: float = 0.45           # blob area / its box area
    shape_min_aspect: float = 1.6         # of the minimum-area rectangle
    shape_max_fill: float = 0.88          # clothing area / that rectangle
    legs_area_ratio: Tuple[float, float] = (0.03, 1.5)   # blue / clothing
    w_clothing: float = 0.30
    w_head: float = 0.35
    w_shape: float = 0.20
    w_legs: float = 0.15
    strong_confidence: float = 0.55
    weak_confidence: float = 0.30
    border_px: int = 3


@dataclass
class Candidate:
    """One candidate in one frame (full-resolution pixels)."""

    x: int
    y: int
    width: int
    height: int
    confidence: float
    head: bool
    shape: bool
    legs: bool
    touches_border: bool
    clothing_area: float

    @property
    def center(self):
        return self.x + self.width / 2.0, self.y + self.height / 2.0

    @property
    def strong(self):
        return self.confidence >= DetectorParams().strong_confidence

    @property
    def short_side(self):
        return min(self.width, self.height)


def to_bgr(data, height, width, encoding, step=None):
    """sensor_msgs/Image payload -> BGR uint8 array (no cv_bridge).

    Supports rgb8, bgr8, rgba8, bgra8 and mono8; anything else raises
    ValueError so that a wrong camera configuration is not processed
    silently.
    """
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if encoding not in channels:
        raise ValueError(f'unsupported image encoding: {encoding!r}')
    n = channels[encoding]
    row = step if step else width * n
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < row * height:
        raise ValueError(
            f'image data too short: {buf.size} bytes for {width}x{height} '
            f'{encoding}')
    image = buf[:row * height].reshape(height, row)[:, :width * n].reshape(
        height, width, n)
    if encoding == 'bgr8':
        return image
    if encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def _merge_boxes(boxes, gap_fraction):
    """Union-find over boxes [x0, y0, x1, y1] that nearly touch."""
    parent = list(range(len(boxes)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            gap_x = max(a[0], b[0]) - min(a[2], b[2])
            gap_y = max(a[1], b[1]) - min(a[3], b[3])
            size = min(max(a[2] - a[0], a[3] - a[1]),
                       max(b[2] - b[0], b[3] - b[1]))
            if max(gap_x, gap_y) <= gap_fraction * size:
                parent[find(i)] = find(j)
    groups = {}
    for i in range(len(boxes)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def detect(bgr, params: DetectorParams = DetectorParams()) -> List[Candidate]:
    """Find survivor candidates in one BGR image."""
    full_h, full_w = bgr.shape[:2]
    s = params.scale
    small = bgr if s == 1.0 else cv2.resize(
        bgr, (max(1, int(round(full_w * s))), max(1, int(round(full_h * s)))),
        interpolation=cv2.INTER_AREA)
    h, w = small.shape[:2]
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    clothing = cv2.inRange(
        hsv, (params.clothing_h[0], params.clothing_s_min,
              params.clothing_v_min), (params.clothing_h[1], 255, 255))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    clothing = cv2.morphologyEx(clothing, cv2.MORPH_OPEN, kernel)
    clothing = cv2.morphologyEx(clothing, cv2.MORPH_CLOSE, kernel,
                                iterations=2)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        clothing, connectivity=8)
    min_area = params.min_area_px * s * s
    regions = [i for i in range(1, count)
               if stats[i, cv2.CC_STAT_AREA] >= min_area * 0.25]
    if not regions:
        return []
    boxes = [[stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
              stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH],
              stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT]]
             for i in regions]
    skin = legs = None
    out = []
    for group in _merge_boxes(boxes, params.merge_gap_fraction):
        ids = [regions[k] for k in group]
        area = float(sum(stats[i, cv2.CC_STAT_AREA] for i in ids))
        if area < min_area:
            continue
        x0 = min(boxes[k][0] for k in group)
        y0 = min(boxes[k][1] for k in group)
        x1 = max(boxes[k][2] for k in group)
        y1 = max(boxes[k][3] for k in group)
        grow = int(round(params.search_fraction * max(x1 - x0, y1 - y0)))
        sx0, sy0 = max(0, x0 - grow), max(0, y0 - grow)
        sx1, sy1 = min(w, x1 + grow), min(h, y1 + grow)
        window = hsv[sy0:sy1, sx0:sx1]
        cloth_win = np.isin(labels[sy0:sy1, sx0:sx1], ids).astype(np.uint8)

        # shape: elongated or not box-filling (arms spread from the torso)
        pts = cv2.findNonZero(cloth_win)
        (_, (rw, rh), _) = cv2.minAreaRect(pts)
        long_side, short_side = max(rw, rh, 1.0), max(min(rw, rh), 1.0)
        shape = bool(long_side / short_side >= params.shape_min_aspect
                     and area / (long_side * short_side)
                     <= params.shape_max_fill)

        # head: a compact skin-coloured blob touching the clothing
        skin = cv2.inRange(
            window, (params.skin_h[0], params.skin_s[0], params.skin_v_min),
            (params.skin_h[1], params.skin_s[1], 255))
        skin = cv2.morphologyEx(skin, cv2.MORPH_OPEN, kernel)
        near = cv2.dilate(cloth_win, cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (5, 5)))
        head, head_box = False, None
        n_skin, skin_labels, skin_stats, _ = \
            cv2.connectedComponentsWithStats(skin, connectivity=8)
        best = 0.0
        for j in range(1, n_skin):
            blob_area = float(skin_stats[j, cv2.CC_STAT_AREA])
            ratio = blob_area / area
            if not (params.head_area_ratio[0] <= ratio
                    <= params.head_area_ratio[1]):
                continue
            bw = skin_stats[j, cv2.CC_STAT_WIDTH]
            bh = skin_stats[j, cv2.CC_STAT_HEIGHT]
            if blob_area / float(bw * bh) < params.head_min_fill:
                continue
            if not np.any((skin_labels == j) & (near > 0)):
                continue
            if blob_area > best:
                best, head = blob_area, True
                bx = skin_stats[j, cv2.CC_STAT_LEFT]
                by = skin_stats[j, cv2.CC_STAT_TOP]
                head_box = (sx0 + bx, sy0 + by, sx0 + bx + bw, sy0 + by + bh)

        # legs: a blue blob touching the clothing, of plausible size
        legs = cv2.inRange(
            window, (params.legs_h[0], params.legs_s_min, params.legs_v_min),
            (params.legs_h[1], 255, 255))
        legs = cv2.morphologyEx(legs, cv2.MORPH_OPEN, kernel)
        has_legs, legs_box = False, None
        n_legs, legs_labels, legs_stats, _ = \
            cv2.connectedComponentsWithStats(legs, connectivity=8)
        for j in range(1, n_legs):
            ratio = float(legs_stats[j, cv2.CC_STAT_AREA]) / area
            if not (params.legs_area_ratio[0] <= ratio
                    <= params.legs_area_ratio[1]):
                continue
            if not np.any((legs_labels == j) & (near > 0)):
                continue
            has_legs = True
            lx = sx0 + legs_stats[j, cv2.CC_STAT_LEFT]
            ly = sy0 + legs_stats[j, cv2.CC_STAT_TOP]
            box = (lx, ly, lx + legs_stats[j, cv2.CC_STAT_WIDTH],
                   ly + legs_stats[j, cv2.CC_STAT_HEIGHT])
            legs_box = box if legs_box is None else (
                min(legs_box[0], box[0]), min(legs_box[1], box[1]),
                max(legs_box[2], box[2]), max(legs_box[3], box[3]))

        if head_box is not None:
            x0, y0 = min(x0, head_box[0]), min(y0, head_box[1])
            x1, y1 = max(x1, head_box[2]), max(y1, head_box[3])
        if legs_box is not None:
            x0, y0 = min(x0, legs_box[0]), min(y0, legs_box[1])
            x1, y1 = max(x1, legs_box[2]), max(y1, legs_box[3])

        confidence = (params.w_clothing + params.w_head * head
                      + params.w_shape * shape + params.w_legs * has_legs)
        if confidence < params.weak_confidence:
            continue
        fx0, fy0 = int(round(x0 / s)), int(round(y0 / s))
        fx1 = min(full_w, int(round(x1 / s)))
        fy1 = min(full_h, int(round(y1 / s)))
        border = params.border_px
        out.append(Candidate(
            x=fx0, y=fy0, width=max(1, fx1 - fx0), height=max(1, fy1 - fy0),
            confidence=float(round(confidence, 3)), head=head, shape=shape,
            legs=has_legs,
            touches_border=bool(fx0 <= border or fy0 <= border
                                or fx1 >= full_w - border
                                or fy1 >= full_h - border),
            clothing_area=area / (s * s)))
    out.sort(key=lambda c: (c.confidence, c.clothing_area), reverse=True)
    return out
