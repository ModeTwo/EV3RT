import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import math
import numpy as np
from enum import Enum
import time
import threading
import zxingcpp
from .plotter import Plotter
from etrobo_python import Hub, Motor, ColorSensor, SonarSensor, GyroSensor

def round_up_to_odd(f) -> int:
    return int(np.ceil(f / 2.) * 2 + 1)

# frame size for Raspberry Pi USB camera capture
IN_FRAME_WIDTH_QR  = 1920
IN_FRAME_HEIGHT_QR = 1080
IN_FRAME_WIDTH     = 640
IN_FRAME_HEIGHT    = 480
# frame size for OpenCV
FRAME_WIDTH  = 320
FRAME_HEIGHT = 180

# frame size for X11 painting
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 180

# text overlay scale, relative to OUT_FRAME_WIDTH
TEXT_SCALE = OUT_FRAME_WIDTH / 2000.0

# constants for TargetInterested.LINE
CROP_WIDTH     = int(15*FRAME_WIDTH/16)
CROP_HEIGHT    = int(3*FRAME_HEIGHT/8)
CROP_U_LIMIT   = FRAME_HEIGHT-CROP_HEIGHT
CROP_D_LIMIT   = FRAME_HEIGHT
CROP_L_LIMIT   = int((FRAME_WIDTH-CROP_WIDTH)/2)
CROP_R_LIMIT   = (CROP_L_LIMIT+CROP_WIDTH)
MORPH_KERNEL_SIZE = round_up_to_odd(int(FRAME_WIDTH/48))
ROI_BOUNDARY   = int(FRAME_WIDTH/10)
LINE_THICKNESS = int(FRAME_WIDTH/160)
CIRCLE_RADIUS  = int(FRAME_WIDTH/80)
SCAN_V_POS     = int(16*FRAME_HEIGHT/16 - LINE_THICKNESS)

HORIZON_DISTANCE = 270 # length of the closest horizontal line on ground within the camera vision
AXLE_TO_HORIZON_DISTANCE = 230 # distance from axle to the closest horizontal line on ground the camera can see

SCAN_BAND_TOP     = CROP_U_LIMIT # highest row the band may climb to
ROI_HOLD_FRAMES   = 3            # keep last ROI this long before full-crop reset
ROE_DEGEN         = 90           # span above this = line ~tangent, cx unusable
CURV_COMP_GAIN    = 8.0   # THE tunable. 0 == today's behavior (no compensation).
CURV_MIN_ROWS_SEP = 15    # need this many rows between near/far to trust the slope
CURV_BAND_ROWS    = 40    # cap the curvature baseline -> keep the estimate local
CURV_MAX_BIAS     = 60    # px clamp on the applied outward bias

# constants for TargetInterested.BOTTLE
BOTTLE_MIN_AREA         = 150   # px^2 contour-area floor (reject specks)
BOTTLE_MIN_EXTENT       = 0.45  # area / bbox-area; a tape band is fairly solid
BOTTLE_BLACK_MAX_W      = int(FRAME_WIDTH * 0.55)  # black blob wider than this = the line
BOTTLE_BLACK_MAX_ASPECT = 4.0   # w/h; the line is far more elongated than a band
BOTTLE_BLIND_ROW        = FRAME_HEIGHT - 4  # band bottom at/below this row => the band is
                                            # crossing into the camera blind spot (~220 mm)

# constants for TargetInterested.QRCODE
CROP_X1   = 360
CROP_X2   = 1560
QR_ROI_MARGIN = 40
QR_LINE_THICKNESS = int(IN_FRAME_WIDTH / 160)   # = 12
TEXT_EXPIRY_SEC = 2.0     # clear decoded text after this many seconds
# internal constants for QR detection and decode pipeline
_QR_ONLY = zxingcpp.BarcodeFormat.QRCode
_GH      = zxingcpp.Binarizer.GlobalHistogram
_WECHAT  = cv2.wechat_qrcode_WeChatQRCode()

_CL_DETECT = cv2.createCLAHE(clipLimit=3, tileGridSize=(8, 8))
_CL_ROI = [
    cv2.createCLAHE(clipLimit=10, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(6, 6)),
    cv2.createCLAHE(clipLimit=20, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=40, tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=3,  tileGridSize=(4, 4)),
    cv2.createCLAHE(clipLimit=6,  tileGridSize=(4, 4)),
]

class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"

class TargetInterested(Enum):
    LINE = "Line"
    QRCODE = "QR Code"
    BOTTLE = "Bottle"

# capture format per interest: (fourcc, width, height, fps)
# MJPG = compressed, high fps for line tracing; YUYV = uncompressed, fine
# detail for small QR codes (low fps at 1080p over USB is expected).
_CAP_CONFIG = {
    TargetInterested.LINE:   ("MJPG", IN_FRAME_WIDTH, IN_FRAME_HEIGHT, 30),
    TargetInterested.BOTTLE: ("MJPG", IN_FRAME_WIDTH, IN_FRAME_HEIGHT, 30),
    TargetInterested.QRCODE: ("YUYV", IN_FRAME_WIDTH_QR, IN_FRAME_HEIGHT_QR, 5),
}

class BottleColor(Enum):
    NONE   = "None"
    RED    = "Red"
    BLUE   = "Blue"
    YELLOW = "Yellow"
    BLACK  = "Black"

# HSV (OpenCV: H 0-179, S/V 0-255) gates per tape colour. RED needs two ranges
# because its hue wraps across 0/180. RED is the only sample-tuned entry; the
# others are tentative and must be re-measured once real samples exist.
BOTTLE_HSV = {
    BottleColor.RED:    [((  0, 120,  70), ( 10, 255, 255)),
                         ((170, 120,  70), (179, 255, 255))],
    BottleColor.BLUE:   [((100, 100,  60), (130, 255, 255))],   # tentative
    BottleColor.YELLOW: [(( 20, 100,  80), ( 35, 255, 255))],   # tentative
    BottleColor.BLACK:  [((  0,   0,   0), (179, 120,  60))],   # tentative; shape-gated
}

class Video(object):
    def __init__(self):
        cv2.setLogLevel(3) # LOG_LEVEL_WARNING
        # set number of threads
        #cv2.setNumThreads(0)
        # prepare the camera
        self.cap = None
        self._cap_cfg = None
        self._pending_lock = threading.Lock()
        self._pending_cap_cfg = None
        # initial mode is LINE → open in MJPG
        self._open_cap(*_CAP_CONFIG[TargetInterested.LINE])
        self.target_interested = TargetInterested.LINE

        # ----- internal state for TargetInterested.LINE        
        # initial region of interest
        self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
        # prepare and keep kernel for morphology
        self.kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
        # initial trace target
        self.cx = int(FRAME_WIDTH/2)
        self.cy = SCAN_V_POS
        self.mx = self.cx
        self._blind_frames = 0          # consecutive frames with no contour
        # default values
        self.gsmin = 0
        self.gsmax = 50
        self.line_tilt = 0.0     # band slope (px/row); cut symptom for delay tuning
        self.band_sep  = 0       # rows between near/far clean samples; 0 = band collapsed
        self.trace_side = TraceSide.NORMAL
        self.range_of_edges = 0
        self.theta:float = 0.0

        # ----- internal state for TargetInterested.BOTTLE
        self._bottle_lock_color = None          # None = auto-scan all colours
        self.bottle_color  = BottleColor.NONE
        self.bottle_cx     = int(FRAME_WIDTH/2)
        self.bottle_theta  = 0.0
        self.bottle_bottom_row = 0
        self.bottle_area   = 0
        self._bottle_lock  = threading.Lock()
        # (insight, color, cx, theta, bottom_row, area, in_blindspot)
        self._bottle_stamped = (False, BottleColor.NONE, int(FRAME_WIDTH/2), 0.0, 0, 0, False)

        # ----- frame id as the common key across the producer and consumers in logging
        self.frame_id = 0
        self._theta_lock = threading.Lock()
        self._theta_stamped = (0.0, 0, 0.0, 0)   # (theta, frame_id, capture_time, odo_mm)

        # ----- shared state between capture thread and detection thread for TargetInterested.QRCODE
        self._frame_lock  = threading.Lock()
        self._latest_gray = None   # latest grayscale frame for detection thread to consume
        self._result_lock      = threading.Lock()
        self._detected_text    = ""    # last successfully decoded QR text
        self._detected_corners = None  # corners of most recently located QR (full-image coords)
        self._is_detecting     = False # True while detection thread is busy
        self._last_decode_time = 0.0   # time.time() of last successful decode

        self.target_insight = False

    def __del__(self):
        cv2.destroyAllWindows()
        self.cap.release()

    def _open_cap(self, fourcc, width, height, fps):
        """(Re)open the V4L2 capture in the given pixel format.
        MUST run on the capture thread (called from __init__ before threads
        start, and from the top of process()). Never call from a behavior thread."""
        if self.cap is not None:
            self.cap.release()
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # FOURCC first: setting resolution can reset the format and vice-versa.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

        if not cap.isOpened():
            print("WARN cap failed to open for fourcc=%s" % fourcc)

        # verify what the driver actually applied
        got = int(cap.get(cv2.CAP_PROP_FOURCC))
        got_str = "".join(chr((got >> 8 * i) & 0xFF) for i in range(4))
        if got_str != fourcc:
            print("WARN cap fourcc requested=%s got=%s" % (fourcc, got_str))

        got_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = cap.get(cv2.CAP_PROP_FPS)
        if (got_w, got_h) != (width, height):
            print("WARN cap size requested=%dx%d got=%dx%d" % (width, height, got_w, got_h))

        for _ in range(5):          # discard warm-up frames while AE/format settle
            cap.read()
        self.cap = cap
        self._cap_cfg = (fourcc, width, height, fps)
        print("VID cap opened fourcc=%s req=%dx%d@%d got=%dx%d@%.1f" % (
            got_str, width, height, fps, got_w, got_h, got_fps))

    def _result_pos_to_corners(self, r):
        """Return 4 corner points (x, y) in crop coordinates, or None."""
        pos = r.position
        return [
            (pos.top_left.x,     pos.top_left.y),
            (pos.top_right.x,    pos.top_right.y),
            (pos.bottom_right.x, pos.bottom_right.y),
            (pos.bottom_left.x,  pos.bottom_left.y),
        ]

    def _extract_roi(self, result, crop):
        pts = self._result_pos_to_corners(result)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x1, x2 = min(xs), max(xs)
        y1, y2 = min(ys), max(ys)
        return crop[
            max(0, y1 - QR_ROI_MARGIN) : y2 + QR_ROI_MARGIN,
            max(0, x1 - QR_ROI_MARGIN) : x2 + QR_ROI_MARGIN,
        ]

    def _wechat_decode_roi(self, roi):
        for cl in _CL_ROI:
            texts, _ = _WECHAT.detectAndDecode(cl.apply(roi))
            t = next((s for s in texts if s), None)
            if t:
                return t
        return ""

    def _detect_qr(self, img_gray):
        """Detect and decode QR from a grayscale 1920x1080 image.

        Returns (text, corners) where:
        text    -- decoded string, or "" if not decoded
        corners -- list of 4 (x, y) in full-image coords if QR located, else None
        """
        crop = img_gray[:, CROP_X1:CROP_X2]

        # Stage 1: GH fast path (QRv1 mostly)
        codes = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, binarizer=_GH)
        if codes:
            r = codes[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            return r.text, corners

        # Stage 2: LA + return_errors to get QRv6 position
        results = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, return_errors=True)
        if results:
            r = results[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            if r.valid:
                return r.text, corners
            text = self._wechat_decode_roi(self._extract_roi(r, crop))
            return text, corners

        # Stage 3: CLAHE detection (zero false-positive rate on noise/part)
        results = zxingcpp.read_barcodes(
            _CL_DETECT.apply(crop), formats=_QR_ONLY, return_errors=True
        )
        if results:
            r = results[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            if r.valid:
                return r.text, corners
            text = self._wechat_decode_roi(self._extract_roi(r, crop))
            return text, corners

        return "", None

    def _detection_worker(self) -> None:
        """Background thread: continuously processes the latest frame."""
        while True:
            # Grab latest frame
            with self._frame_lock:
                gray = self._latest_gray
                self._latest_gray = None  # mark as consumed

            if gray is None:
                time.sleep(0.005)
                continue

            with self._result_lock:
                self._is_detecting = True

            text, corners = self._detect_qr(gray)

            with self._result_lock:
                self._is_detecting = False
                self._detected_corners = corners
                if text:
                    self._detected_text = text
                    self._last_decode_time = time.time()


    def process(self,
                plotter: Plotter,
                hub: Hub,
                arm_motor: Motor,
                right_motor: Motor,
                left_motor: Motor,
                color_sensor: ColorSensor,
                sonar_sensor: SonarSensor,
                gyro_sensor: GyroSensor) -> None:

        # apply any pending capture-format switch on THIS (capture) thread
        with self._pending_lock:
            cfg = self._pending_cap_cfg
            self._pending_cap_cfg = None
        if cfg is not None:
            self._open_cap(*cfg)

        ret, frame = self.cap.read()

        if frame is None:
            cv2.waitKey(1)
            return
        t_cap = time.time()          # capture time for this frame
        self.frame_id += 1

        if self.target_interested == TargetInterested.QRCODE:
            # ---- QR needs the original full-resolution image ----
            img_orig = frame.copy()
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            with self._frame_lock:
                self._latest_gray = img_gray

            # Read detection state; expire text after TEXT_EXPIRY_SEC
            with self._result_lock:
                if self._detected_text and (time.time() - self._last_decode_time) > TEXT_EXPIRY_SEC:
                    self._detected_text = ""
                qr_text   = self._detected_text
                corners   = self._detected_corners
                detecting = self._is_detecting

            # Gray out regions outside the crop (x < CROP_X1 and x > CROP_X2)
            gray3 = cv2.cvtColor(cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            img_orig[:, :CROP_X1]  = gray3[:, :CROP_X1]
            img_orig[:, CROP_X2:]  = gray3[:, CROP_X2:]

            # Draw bounding box around located QR (in full-image coords)
            if corners is not None:
                self.target_insight = True
                color = (0, 255, 0) if qr_text else (0, 200, 255)
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(img_orig, [pts], isClosed=True, color=color, thickness=LINE_THICKNESS)
            else:
                self.target_insight = False

        elif self.target_interested == TargetInterested.BOTTLE:
            # ---- BOTTLE runs on the downsized frame ----
            # 640x480 (4:3) capture -> center-crop to 16:9 -> 320x180.
            # Cropping (not squashing) preserves the pixel->angle/mm geometry
            # that SCAN_V_POS, theta, roe, tilt all depend on.
            if self.frame_id == 1:
                print("VID first BOTTLE frame shape=%s" % (frame.shape,))
            fh, fw = frame.shape[:2]
            crop_h = int(fw * 9 / 16)             # 360 for a 640-wide frame
            y0 = (fh - crop_h) // 2               # center band
            frame_169 = frame[y0:y0 + crop_h, :]
            img_orig = cv2.resize(frame_169, (FRAME_WIDTH, FRAME_HEIGHT))
            img_hsv  = cv2.cvtColor(img_orig, cv2.COLOR_BGR2HSV)
            # Track the locked colour once identified, else scan all four.
            if self._bottle_lock_color is not None:
                candidates = [self._bottle_lock_color]
            else:
                candidates = [BottleColor.RED, BottleColor.BLUE,
                              BottleColor.YELLOW, BottleColor.BLACK]

            best = None   # (area, color, cx, bottom_row, (x,y,w,h), cnt)
            for color in candidates:
                mask = self._bottle_mask(img_hsv, color)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in cnts:
                    area = cv2.contourArea(cnt)
                    if area < BOTTLE_MIN_AREA:
                        continue
                    x, y, w, h = cv2.boundingRect(cnt)
                    extent = area / float(w * h)
                    aspect = w / float(h) if h > 0 else 999.0
                    # A BLACK band must be told apart from the course line: the
                    # line is long, thin and spans much of the width; the band is
                    # a compact, fairly solid blob.
                    if color == BottleColor.BLACK:
                        if w > BOTTLE_BLACK_MAX_W:           continue
                        if aspect > BOTTLE_BLACK_MAX_ASPECT: continue
                        if extent < BOTTLE_MIN_EXTENT:       continue
                    if best is None or area > best[0]:
                        best = (area, color, x + w // 2, y + h, (x, y, w, h), cnt)

            if best is not None:
                area, color, bcx, bbottom, (bx, by, bw, bh), cnt = best
                self.target_insight    = True
                self.bottle_color      = color
                self.bottle_cx         = bcx
                self.bottle_area       = int(area)
                self.bottle_bottom_row = bbottom
                # bearing to the band, reusing the LINE pixel->angle conversion
                vxp = bcx - int(FRAME_WIDTH / 2)
                vxm = vxp * HORIZON_DISTANCE / FRAME_WIDTH
                self.bottle_theta = 180 * math.atan(vxm / AXLE_TO_HORIZON_DISTANCE) / math.pi
                in_blind = bbottom >= BOTTLE_BLIND_ROW
                # feed the shared text/overlay block below
                self.cx, self.cy, self.theta = bcx, bbottom, self.bottle_theta
                col = {BottleColor.RED:(0,0,255), BottleColor.BLUE:(255,0,0),
                       BottleColor.YELLOW:(0,255,255), BottleColor.BLACK:(60,60,60)}[color]
                cv2.rectangle(img_orig, (bx,by), (bx+bw, by+bh), col, LINE_THICKNESS)
                cv2.drawContours(img_orig, [cnt], 0, (0,255,0), 1)
                if in_blind:
                    cv2.line(img_orig, (0, BOTTLE_BLIND_ROW),
                             (FRAME_WIDTH, BOTTLE_BLIND_ROW), (0,0,255), 1)
            else:
                self.target_insight    = False
                self.bottle_area       = 0
                self.bottle_bottom_row = 0
                in_blind = False

            with self._bottle_lock:
                self._bottle_stamped = (self.target_insight, self.bottle_color,
                                        self.bottle_cx, self.bottle_theta,
                                        self.bottle_bottom_row, self.bottle_area, in_blind)

        else: # TargetInterested.LINE
            # ---- LINE runs on the downsized frame ----
            # 640x480 (4:3) capture -> center-crop to 16:9 -> 320x180.
            # Cropping (not squashing) preserves the pixel->angle/mm geometry
            # that SCAN_V_POS, theta, roe, tilt all depend on.
            if self.frame_id == 1:
                print("VID first LINE frame shape=%s" % (frame.shape,))
            fh, fw = frame.shape[:2]
            crop_h = int(fw * 9 / 16)             # 360 for a 640-wide frame
            y0 = (fh - crop_h) // 2               # center band
            frame_169 = frame[y0:y0 + crop_h, :]
            img_orig = cv2.resize(frame_169, (FRAME_WIDTH, FRAME_HEIGHT))
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            # crop a part of image for binarization
            img_gray_part = img_gray[CROP_U_LIMIT:CROP_D_LIMIT, CROP_L_LIMIT:CROP_R_LIMIT]
            # binarize the image
            img_bin_part = cv2.inRange(img_gray_part, self.gsmin, self.gsmax)
            # prepare an empty matrix
            img_bin = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
            # copy img_bin_part into img_bin
            img_bin[CROP_U_LIMIT:CROP_U_LIMIT+img_bin_part.shape[0], CROP_L_LIMIT:CROP_L_LIMIT+img_bin_part.shape[1]] = img_bin_part
            # remove noise
            img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernel)
            # convert the binary image from grayscale to BGR for later
            #img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

            # focus on the region of interest
            x, y, w, h = self.roi
            img_roi = img_bin_mor[y:y+h, x:x+w]
            # find contours in the roi with offset
            contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
            # identify the largest contour
            if len(contours) >= 1:
                i_target = 0
                if len(contours) >= 2:
                    cnt_idx = np.empty((0,2), int)
                    for i, cnt in enumerate(contours):
                        area = cv2.contourArea(cnt)
                        cnt_idx = np.append(cnt_idx, np.array([[i, int(area)]]), axis=0)
                    # sort cnt_idx by area in descending order
                    cnt_idx = cnt_idx[np.argsort(cnt_idx[:, 1])[::-1]]
                    # calculate the bounding box around the two largest contours,
                    # either of which is to be set as the new region of interest 
                    x1, y1, w1, h1 = cv2.boundingRect(contours[cnt_idx[0,0]])
                    x2, y2, w2, h2 = cv2.boundingRect(contours[cnt_idx[1,0]])
                    x, y, w, h = self.roi
                    # the second largest contour touches the roi bottom
                    # and the second largest contour is large enough?
                    if y2 + h2 >= h and 2*cnt_idx[1,1] >= cnt_idx[0,1]:
                        if self.trace_side == TraceSide.LEFT:
                            # look for the left rectangle
                            if x1 <= x2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        elif self.trace_side == TraceSide.RIGHT:
                            # look for the right rectangle
                            if x1 + w1 >= x2 + w2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        else: # tracing the line center goes after the largest contour
                            i_target = cnt_idx[0,0]
                    else: # the second largest contour does not touch the roi bottom and shall be ignored
                        i_target = cnt_idx[0,0]

                # draw the target contour on the original image
                img_orig = cv2.polylines(img_orig, [contours[i_target]], 0, (0,255,0), LINE_THICKNESS)

                # calculate the bounding box around the largest contour
                # and set it as the new region of interest
                x, y, w, h = cv2.boundingRect(contours[i_target])
                # adjust the region of interest
                x = x - ROI_BOUNDARY
                y = y - ROI_BOUNDARY
                w = w + 2*ROI_BOUNDARY
                h = h + 2*ROI_BOUNDARY
                if x < CROP_L_LIMIT:
                    x = CROP_L_LIMIT
                if y < CROP_U_LIMIT:
                    y = CROP_U_LIMIT
                if x + w > CROP_R_LIMIT:
                    w = CROP_R_LIMIT - x
                if y + h > CROP_D_LIMIT:
                    h = CROP_D_LIMIT - y
                # set the new region of interest
                self.roi = (x, y, w, h)
            
                # prepare for trace target calculation
                img_cnt = np.zeros_like(img_orig)
                img_cnt = cv2.drawContours(img_cnt, [contours[i_target]], 0, (0,255,0), 1)
                img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)

                # ---- banded scan + live curvature pre-compensation ----
                # Bottom (nearest) row still defines range_of_edges for IsJunction.
                b_edges = np.flatnonzero(img_cnt_gray[SCAN_V_POS])
                if len(b_edges) >= 2:
                    self.range_of_edges = int(b_edges[-1] - b_edges[0])
                elif len(b_edges) == 1:
                    self.range_of_edges = 1
                else:
                    self.range_of_edges = 0

                # Collect the line center at every CLEAN (non-tangent) height of the
                # band, bottom row first. Tangent rows (span > ROE_DEGEN) are skipped,
                # so they can neither be the steering target nor pollute the slope.
                samples = []   # (row, cx_row), nearest first
                for row in range(SCAN_V_POS, SCAN_BAND_TOP - 1, -1):
                    edges = np.flatnonzero(img_cnt_gray[row])
                    if len(edges) >= 2:
                        if int(edges[-1] - edges[0]) > ROE_DEGEN:
                            continue
                        if self.trace_side == TraceSide.LEFT:
                            cx_row = int(edges[0])
                        elif self.trace_side == TraceSide.RIGHT:
                            cx_row = int(edges[-1])
                        else:
                            cx_row = (int(edges[0]) + int(edges[-1])) // 2
                    elif len(edges) == 1:
                        cx_row = int(edges[0])
                    else:
                        continue
                    samples.append((row, cx_row))

                if samples:
                    near_row, cx_raw = samples[0]      # base target == today's behavior

                    # Pick the farthest clean row WITHIN the local curvature window.
                    # Local-only keeps the "locally circular" assumption honest and
                    # avoids reaching into a diverging branch at a fork.
                    far_row, cx_far = near_row, cx_raw
                    for r, c in samples:
                        if (near_row - r) <= CURV_BAND_ROWS:
                            far_row, cx_far = r, c
                        else:
                            break

                    bias = 0
                    sep = near_row - far_row
                    self.band_sep = sep          # publish for the feed-forward gate

                    # Line tilt across the band (px/row): how far the line drifts
                    # per row from the near (axle-side) row to the far (look-ahead)
                    # row. This is the cut symptom — a robot on the line reads a
                    # small tilt, a robot cutting ~100mm inside reads a steep one.
                    # Measured whenever the band is long enough, regardless of
                    # whether the curvature BIAS is gated off below.
                    self.line_tilt = ((cx_raw - cx_far) / float(sep)
                                      if sep >= CURV_MIN_ROWS_SEP else 0.0)

                    bottom_clean = (0 < self.range_of_edges <= ROE_DEGEN)
                    WALL_MARGIN = 30
                    near_wall = (cx_raw <= CROP_L_LIMIT + WALL_MARGIN or
                                 cx_raw >= CROP_R_LIMIT - WALL_MARGIN or
                                 cx_far <= CROP_L_LIMIT + WALL_MARGIN or
                                 cx_far >= CROP_R_LIMIT - WALL_MARGIN)
                    if sep >= CURV_MIN_ROWS_SEP and bottom_clean and not near_wall:
                        slope = self.line_tilt
                        bias  = int(CURV_COMP_GAIN * slope)
                        bias  = max(-CURV_MAX_BIAS, min(CURV_MAX_BIAS, bias))

                    cx_comp = max(CROP_L_LIMIT, min(CROP_R_LIMIT, cx_raw + bias))
                    self.cx = cx_comp
                    self.mx = self.cx
                    self._blind_frames = 0
                    self.target_insight = True

                    # tuning log: raw vs compensated, the bias, and how the estimate
                    # was formed (clean-row count and the baseline used)
                    #if bias != 0:
                    #    print("%+06d CRV fid=%06d cx_raw=%03d bias=%+03d cx=%03d n=%02d sep=%02d" % (
                    #        plotter.get_distance() if plotter is not None else 0,
                    #        self.frame_id, cx_raw, bias, self.cx, len(samples), sep))
                else:
                    # whole band tangent/empty: target meaningless -> hold heading.
                    self.target_insight = False

            else: # len(contours) == 0
                self._blind_frames += 1
                self.range_of_edges = 0
                self.line_tilt = 0.0
                self.band_sep = 0
                if self._blind_frames <= ROI_HOLD_FRAMES:
                    # brief dropout (e.g. bottom-row tangent on a sharp turn):
                    # don't reset to full crop, but GROW the last ROI outward each
                    # blind frame so a line that slipped just past the box edge is
                    # re-acquired where it actually went, not where it was.
                    x, y, w, h = self.roi
                    x -= ROI_BOUNDARY; y -= ROI_BOUNDARY
                    w += 2*ROI_BOUNDARY; h += 2*ROI_BOUNDARY
                    if x < CROP_L_LIMIT: x = CROP_L_LIMIT
                    if y < CROP_U_LIMIT: y = CROP_U_LIMIT
                    if x + w > CROP_R_LIMIT: w = CROP_R_LIMIT - x
                    if y + h > CROP_D_LIMIT: h = CROP_D_LIMIT - y
                    self.roi = (x, y, w, h)
                else:
                    # sustained loss: fall back to full-crop search
                    self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
                # keep mx (heading) so the robot maintains its current move;
                # mirror cx to mx so the two stay coherent on re-acquisition
                self.cx = self.mx
                self.cy = SCAN_V_POS
                self.target_insight = False
            
            # draw the area of interest on the original image
            x, y, w, h = self.roi
            if self.target_insight:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
            else:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (0,0,255), LINE_THICKNESS)
            # draw the trace target on the image
            cv2.circle(img_orig, (self.mx, SCAN_V_POS), CIRCLE_RADIUS, (0,0,255), -1)
            # calculate variance of mx from the center in pixel
            vxp = self.mx - int(FRAME_WIDTH/2)
            # convert the variance from pixel to milimeters
            # HORIZON_DISTANCE is length of the closest horizontal line on ground within the camera vision
            vxm = vxp * HORIZON_DISTANCE / FRAME_WIDTH
            # calculate the rotation in radians (z-axis)
            # AXLE_TO_HORIZON_DISTANCE is distance from axle to the closest horizontal line on ground the camera can see
            self.theta = 180 * math.atan(vxm / AXLE_TO_HORIZON_DISTANCE) / math.pi
            # publish theta with its frame id + capture time (consumed by TraceLineCam)
            dist = plotter.get_distance() if plotter is not None else 0
            with self._theta_lock:
                self._theta_stamped = (self.theta, self.frame_id, t_cap, dist)
            #print(
            #    "%+06d VID fid=%06d cap=%.3f cx=%d mx=%d roi=(%d,%d,%d,%d) roe=%03d theta=%+06.1f insight=%d lat=%.1f" % (
            #        dist, self.frame_id, t_cap, self.cx, self.mx,
            #        x, y, w, h, self.range_of_edges, self.theta,
            #        int(self.target_insight), (time.time() - t_cap) * 1000))

        # BELOW IS COMMON FOR ALL TARGETS
        # shrink the processed image straight to the monitor (transmission) size.
        # img_orig may be IN_FRAME (QR branch) or FRAME_* (LINE/BOTTLE); resizing
        # directly to OUT_FRAME means no per-branch upscaling is needed.
        img_mon = cv2.resize(img_orig, (OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT))

        # prepare text area at the monitor width (must match img_mon width for vconcat)
        img_text = np.zeros((OUT_FRAME_HEIGHT, OUT_FRAME_WIDTH, 3), np.uint8)
        ts     = TEXT_SCALE
        f_norm = 1.8 * ts          # normal line font scale
        f_fid  = 2.6 * ts          # FID font scale
        if plotter is not None:
            try:
                cv2.putText(img_text, f"ODO={plotter.get_distance():+06}", (0,int(60*ts)),  cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"x={plotter.get_loc_x():+05} y={plotter.get_loc_y():+05}", (0,int(120*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"gyro={gyro_sensor.get_angle():+04}", (0,int(180*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"cx={self.cx:} cy={self.cy} theta={self.theta:+06.1f}", (0,int(240*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"roe={self.range_of_edges:03}", (0,int(300*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                h, s, v = color_sensor.get_raw_color_hsv()
                cv2.putText(img_text, f"h={h:03} s={s:03} v={v:03}", (0,int(360*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"mV={hub.get_battery_voltage():04} mA={hub.get_battery_current():04}", (0,int(420*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"QR={self.get_QR_text()}", (0,int(480*ts)), cv2.FONT_HERSHEY_DUPLEX, f_norm, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"FID={self.frame_id:06}", (0,int(1000*ts)), cv2.FONT_HERSHEY_DUPLEX, f_fid, (0,255,255), 1, cv2.LINE_AA)
            except Exception as e:
                pass
        # concatenate the images - shrunk original + text area
        img_comm = cv2.vconcat([img_mon, img_text])
        # transmit and display the image
        cv2.imshow("video monitor", img_comm)
        cv2.waitKey(1) # show the window
        return
        
    def get_theta(self) -> float:
        return self.theta

    def get_theta_stamped(self):
        with self._theta_lock:
            return self._theta_stamped   # (theta, frame_id, capture_time, odo_mm)

    def get_line_tilt(self) -> float:
        return self.line_tilt

    def get_band_sep(self) -> int:
        return self.band_sep

    def get_range_of_edges(self) -> int:
        return self.range_of_edges

    def _bottle_mask(self, img_hsv, color):
        mask = None
        for lo, hi in BOTTLE_HSV[color]:
            m = cv2.inRange(img_hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        return mask

    def get_bottle_stamped(self):
        with self._bottle_lock:
            return self._bottle_stamped  # (insight, color, cx, theta, bottom_row, area, in_blind)

    def get_bottle_color(self) -> 'BottleColor':
        with self._bottle_lock:
            return self._bottle_stamped[1]

    def set_bottle_color(self, color) -> None:
        self._bottle_lock_color = color

    def get_QR_text(self) -> str:
        if self.target_interested == TargetInterested.QRCODE:
            with self._result_lock:
                return self._detected_text
        else:
            # latch the detected text until set_target_interest(TargetInterested.QRCODE) is invoked,
            # or TEXT_EXPIRY_SEC is exhausted while self.target_interested == TargetInterested.QRCODE.
            return self._detected_text

    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        self.gsmin = gs_min
        self.gsmax = gs_max
        return

    def set_trace_side(self, trace_side: TraceSide) -> None:
        self.trace_side = trace_side
        return

    def set_target_interested(self, target_interested: TargetInterested) -> None:
        self.target_interested = target_interested

        # request the matching capture format; the reopen itself happens on the
        # capture thread, at the top of process()
        cfg = _CAP_CONFIG.get(target_interested)
        if cfg is not None and cfg != self._cap_cfg:
            with self._pending_lock:
                self._pending_cap_cfg = cfg

        if self.target_interested == TargetInterested.QRCODE and not hasattr(self, "_detection_thread"):
            self._detected_text    = ""    # initialize the detected text for safety
            # Start detection thread
            self._detection_thread = threading.Thread(target=self._detection_worker, daemon=True)
            self._detection_thread.start()
        else:
            # Stop detection thread if not interested in QR code
            if hasattr(self, "_detection_thread"):
                self._detection_thread.join(timeout=1.0)
                del self._detection_thread
        return

    def is_target_insight(self) -> bool:
        return self.target_insight
