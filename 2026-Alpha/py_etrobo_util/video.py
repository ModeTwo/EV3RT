# 【日本語解説】 Raspberry PiからSPIKEとWebカメラを連携させ、ETロボコン2026の走行・画像認識を制御する。
# 【日本語解説】 行動木の各update()は短時間で1周期だけ処理し、完了まではRUNNINGを返す。
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

# 【日本語解説】 入力値以上で最小の奇数を返し、OpenCVのカーネルサイズ条件を満たす。
def round_up_to_odd(f) -> int:
    # 【引数】 f: 奇数へ切り上げる数値。
    return int(np.ceil(f / 2.) * 2 + 1)

# カメラから取得したフレームを画像認識用に処理する
IN_FRAME_WIDTH_QR  = 1920
IN_FRAME_HEIGHT_QR = 1080
IN_FRAME_WIDTH     = 640
IN_FRAME_HEIGHT    = 480
# 後続の画像認識で使用するフレームを生成または更新する。
FRAME_WIDTH  = 320
FRAME_HEIGHT = 180

# 後続の画像認識で使用するフレームを生成または更新する。
OUT_FRAME_WIDTH  = 320
OUT_FRAME_HEIGHT = 180

# TEXT_SCALEへ後続処理で使用する計算結果を保存する。
TEXT_SCALE = OUT_FRAME_WIDTH / 2000.0

# CROP_WIDTHへ後続処理で使用する計算結果を保存する。
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

HORIZON_DISTANCE = 270  # HORIZON_DISTANCEへ、この処理で使用する設定値または計算結果を保存する。
AXLE_TO_HORIZON_DISTANCE = 230  # AXLE_TO_HORIZON_DISTANCEへ、この処理で使用する設定値または計算結果を保存する。

SCAN_BAND_TOP     = CROP_U_LIMIT  # SCAN_BAND_TOPへ、この処理で使用する設定値または計算結果を保存する。
ROI_HOLD_FRAMES   = 3            # 後続処理で使用する状態または設定値を更新する
ROE_DEGEN         = 90  # ラインが画像の接線方向に近いとみなす両端幅のしきい値。
CURV_COMP_GAIN    = 8.0  # CURV_COMP_GAINへ、この処理で使用する設定値または計算結果を保存する。
CURV_MIN_ROWS_SEP = 15  # 曲率推定を有効とする近側・遠側走査行の最小間隔。
CURV_BAND_ROWS    = 40  # CURV_BAND_ROWSへ、この処理で使用する設定値または計算結果を保存する。
CURV_MAX_BIAS     = 60  # CURV_MAX_BIASへ、この処理で使用する設定値または計算結果を保存する。

# BOTTLE_MIN_AREAへ後続処理で使用する計算結果を保存する。
BOTTLE_MIN_AREA         = 150  # BOTTLE_MIN_AREAへ、この処理で使用する設定値または計算結果を保存する。
BOTTLE_MIN_EXTENT       = 0.45  # BOTTLE_MIN_EXTENTへ、この処理で使用する設定値または計算結果を保存する。
BOTTLE_BLACK_MAX_W      = int(FRAME_WIDTH * 0.55)  # BOTTLE_BLACK_MAX_Wへ、この処理で使用する設定値または計算結果を保存する。
BOTTLE_BLACK_MAX_ASPECT = 4.0  # BOTTLE_BLACK_MAX_ASPECTへ、この処理で使用する設定値または計算結果を保存する。
BOTTLE_BLIND_ROW        = FRAME_HEIGHT - 4  # BOTTLE_BLIND_ROWへ、この処理で使用する設定値または計算結果を保存する。
                                            # CROP_X1へ後続処理で使用する計算結果を保存する。

# CROP_X1へ後続処理で使用する計算結果を保存する。
CROP_X1   = 360
CROP_X2   = 1560
QR_ROI_MARGIN = 40
QR_LINE_THICKNESS = int(IN_FRAME_WIDTH / 160)  # QR_LINE_THICKNESSへ、この処理で使用する設定値または計算結果を保存する。
TEXT_EXPIRY_SEC = 2.0  # TEXT_EXPIRY_SECへ、この処理で使用する設定値または計算結果を保存する。
# _QR_ONLYへ後続処理で使用する計算結果を保存する。
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

# 【日本語解説】 画像上でラインの左右どちら側を追従するかを表す列挙型。
class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"

# 【日本語解説】 画像処理で現在探索する対象（ライン、QR、ボトルなど）を切り替える列挙型。
class TargetInterested(Enum):
    LINE = "Line"
    QRCODE = "QR Code"
    BOTTLE = "Bottle"

# _CAP_CONFIGへ後続処理で使用する計算結果を保存する。
# MJPGは圧縮形式でライン追従向けの高FPS、YUYVは非圧縮形式でQR認識向けの高精細映像に使用する
# _CAP_CONFIGへ後続処理で使用する計算結果を保存する。
_CAP_CONFIG = {
    TargetInterested.LINE:   ("MJPG", IN_FRAME_WIDTH, IN_FRAME_HEIGHT, 30),
    TargetInterested.BOTTLE: ("MJPG", IN_FRAME_WIDTH, IN_FRAME_HEIGHT, 30),
    TargetInterested.QRCODE: ("YUYV", IN_FRAME_WIDTH_QR, IN_FRAME_HEIGHT_QR, 5),
}

# 【日本語解説】 カメラで識別・回収するボトル色を表す列挙型。
class BottleColor(Enum):
    NONE   = "None"
    RED    = "Red"
    BLUE   = "Blue"
    YELLOW = "Yellow"
    BLACK  = "Black"

# BOTTLE_HSVへ後続処理で使用する計算結果を保存する。
BOTTLE_HSV = {
    BottleColor.RED:    [((  0, 120,  70), ( 10, 255, 255)),
                         ((170, 120,  70), (179, 255, 255))],
    BottleColor.BLUE:   [((100, 100,  60), (130, 255, 255))],  # この行で指定する値の用途を示す。
    BottleColor.YELLOW: [(( 20, 100,  80), ( 35, 255, 255))],  # この行で指定する値の用途を示す。
    BottleColor.BLACK:  [((  0,   0,   0), (179, 120,  60))],  # この行で指定する値の用途を示す。
}

# 【日本語解説】 Webカメラの取得、ライン解析、QR認識、ボトル検出をまとめて管理する画像処理クラス。
class Video(object):
    # 【日本語解説】 Videoの設定値と実行中に保持する状態を初期化する。
    def __init__(self):
        cv2.setLogLevel(3)  # この行で指定する値の用途を示す。
        # 後続処理で使用する状態または設定値を更新する
        #cv2.setNumThreads(0)
        # 後続処理で使用するデータを準備する
        self.cap = None
        self._cap_cfg = None
        self._pending_lock = threading.Lock()
        self._pending_cap_cfg = None
        # 現在の制御状態に必要な値を更新し、次の処理へ進む。
        self._open_cap(*_CAP_CONFIG[TargetInterested.LINE])
        self.target_interested = TargetInterested.LINE

        # 次フレームで対象を効率よく探索する関心領域を更新する。
        self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
        # 後続処理で使用するデータを準備する
        self.kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
        # self.cxへ後続処理で使用する計算結果を保存する。
        self.cx = int(FRAME_WIDTH/2)
        self.cy = SCAN_V_POS
        self.mx = self.cx
        self._blind_frames = 0  # self._blind_framesへ、この処理で使用する設定値または計算結果を保存する。
        # self.gsminへ後続処理で使用する計算結果を保存する。
        self.gsmin = 0
        self.gsmax = 50
        self.line_tilt = 0.0  # self.line_tiltへ、この処理で使用する設定値または計算結果を保存する。
        self.band_sep  = 0  # self.band_sepへ、この処理で使用する設定値または計算結果を保存する。
        self.trace_side = TraceSide.NORMAL
        self.range_of_edges = 0
        self.theta:float = 0.0

        # 指定時は画像判定を待たず、このボトル色へ追跡を固定する。
        self._bottle_lock_color = None          # None = auto-scan all colours
        self.bottle_color  = BottleColor.NONE
        self.bottle_cx     = int(FRAME_WIDTH/2)
        self.bottle_theta  = 0.0
        self.bottle_bottom_row = 0
        self.bottle_area   = 0
        self._bottle_lock  = threading.Lock()
        # self._bottle_stampedへ後続処理で使用する計算結果を保存する。
        self._bottle_stamped = (False, BottleColor.NONE, int(FRAME_WIDTH/2), 0.0, 0, 0, False)

        # 後続の画像認識で使用するフレームを生成または更新する。
        self.frame_id = 0
        self._theta_lock = threading.Lock()
        self._theta_stamped = (0.0, 0, 0.0, 0)  # self._theta_stampedへ、この処理で使用する設定値または計算結果を保存する。

        # 後続の画像認識で使用するフレームを生成または更新する。
        self._frame_lock  = threading.Lock()
        self._latest_gray = None  # 処理済みフレームを破棄し、次のフレーム受渡しを可能にする。
        self._result_lock      = threading.Lock()
        self._detected_text    = ""  # self._detected_textへ、この処理で使用する設定値または計算結果を保存する。
        self._detected_corners = None  # self._detected_cornersへ、この処理で使用する設定値または計算結果を保存する。
        self._is_detecting     = False  # self._is_detectingへ、この処理で使用する設定値または計算結果を保存する。
        self._last_decode_time = 0.0   # time.time() of last successful decode

        self.target_insight = False

    # 【日本語解説】 Videoの破棄時にカメラやバックグラウンド処理の資源を解放する。
    def __del__(self):
        cv2.destroyAllWindows()
        self.cap.release()

    # 【日本語解説】 指定した解像度・FPS・コーデックでカメラを開き、利用可能なキャプチャを返す。
    def _open_cap(self, fourcc, width, height, fps):
        # 【引数】 fourcc: カメラで使用する映像コーデックの4文字コード。
        # 【引数】 width: 取得するカメラ画像の幅（ピクセル）。
        # 【引数】 height: 取得するカメラ画像の高さ（ピクセル）。
        # 【引数】 fps: カメラから取得する1秒当たりのフレーム数。
        """指定した映像形式でカメラを開く。この処理はキャプチャースレッドからだけ呼び出す。"""
        if self.cap is not None:
            self.cap.release()
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # 後続処理で使用する状態または設定値を更新する
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

        if not cap.isOpened():
            print("WARN cap failed to open for fourcc=%s" % fourcc)

        # gotへ後続処理で使用する計算結果を保存する。
        got = int(cap.get(cv2.CAP_PROP_FOURCC))
        got_str = "".join(chr((got >> 8 * i) & 0xFF) for i in range(4))
        if got_str != fourcc:
            print("WARN cap fourcc requested=%s got=%s" % (fourcc, got_str))

        got_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = cap.get(cv2.CAP_PROP_FPS)
        if (got_w, got_h) != (width, height):
            print("WARN cap size requested=%dx%d got=%dx%d" % (width, height, got_w, got_h))

        for _ in range(5):          # 後続処理で使用する状態または設定値を更新する
            cap.read()
        self.cap = cap
        self._cap_cfg = (fourcc, width, height, fps)
        print("VID cap opened fourcc=%s req=%dx%d@%d got=%dx%d@%.1f" % (
            got_str, width, height, fps, got_w, got_h, got_fps))

    # 【日本語解説】 物体検出結果の位置表現を画像上の矩形座標へ変換する。
    def _result_pos_to_corners(self, r):
        # 【引数】 r: ZXingが返したQRコード検出結果。
        """検出位置から切り出し画像座標系の四隅を返す。位置がなければNoneを返す。"""
        pos = r.position
        return [
            (pos.top_left.x,     pos.top_left.y),
            (pos.top_right.x,    pos.top_right.y),
            (pos.bottom_right.x, pos.bottom_right.y),
            (pos.bottom_left.x,  pos.bottom_left.y),
        ]

    # 【日本語解説】 検出矩形を画像範囲内へ収め、QR再解析用の関心領域を切り出す。
    def _extract_roi(self, result, crop):
        # 【引数】 result: 関心領域を切り出す対象のQRコード検出結果。
        # 【引数】 crop: QRコードを探索した切り出し画像。
        pts = self._result_pos_to_corners(result)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        x1, x2 = min(xs), max(xs)
        y1, y2 = min(ys), max(ys)
        return crop[
            max(0, y1 - QR_ROI_MARGIN) : y2 + QR_ROI_MARGIN,
            max(0, x1 - QR_ROI_MARGIN) : x2 + QR_ROI_MARGIN,
        ]

    # 【日本語解説】 切り出した領域へWeChat QRデコーダーを適用して文字列を得る。
    def _wechat_decode_roi(self, roi):
        # 【引数】 roi: QRコードの再認識を行う関心領域画像。
        for cl in _CL_ROI:
            texts, _ = _WECHAT.detectAndDecode(cl.apply(roi))
            t = next((s for s in texts if s), None)
            if t:
                return t
        return ""

    # 【日本語解説】 画像全体と候補領域を段階的に解析し、QR文字列を検出する。
    def _detect_qr(self, img_gray):
        # 【引数】 img_gray: QRコードを検出するグレースケール画像。
        """1920×1080のグレースケール画像からQRコードを検出・復号する。
        
                戻り値は文字列と四隅座標の組。復号できなければ空文字列、位置を取得できなければNoneを返す。
        """
        crop = img_gray[:, CROP_X1:CROP_X2]

        # 指定画像からQRコード候補を検出する。
        codes = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, binarizer=_GH)
        if codes:
            r = codes[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            return r.text, corners

        # 指定画像からQRコード候補を検出する。
        results = zxingcpp.read_barcodes(crop, formats=_QR_ONLY, return_errors=True)
        if results:
            r = results[0]
            corners = [(x + CROP_X1, y) for x, y in self._result_pos_to_corners(r)]
            if r.valid:
                return r.text, corners
            text = self._wechat_decode_roi(self._extract_roi(r, crop))
            return text, corners

        # 指定画像からQRコード候補を検出する。
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

    # 【日本語解説】 共有された最新フレームに対して、重い物体検出処理をバックグラウンド実行する。
    def _detection_worker(self) -> None:
        """最新フレームを継続的に処理するバックグラウンドスレッド。"""
        while True:
            # 現在の制御状態に必要な値を更新し、次の処理へ進む。
            with self._frame_lock:
                gray = self._latest_gray
                self._latest_gray = None  # 処理済みフレームを破棄し、次のフレーム受渡しを可能にする。

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


    # 【日本語解説】 カメラの1フレームを処理し、現在の探索対象に応じた認識結果を共有状態へ反映する。
    def process(self,
                plotter: Plotter,
                hub: Hub,
                arm_motor: Motor,
                right_motor: Motor,
                left_motor: Motor,
                color_sensor: ColorSensor,
                sonar_sensor: SonarSensor,
                gyro_sensor: GyroSensor) -> None:

        # キャプチャースレッド上で保留中の映像形式変更を適用する
        # 【引数】 plotter: 走行距離・方位・座標を推定するオドメトリ管理オブジェクト。
        # 【引数】 hub: SPIKEハブを操作・参照するデバイスオブジェクト。
        # 【引数】 arm_motor: アーム駆動用モーター。
        # 【引数】 right_motor: 右車輪駆動用モーター。
        # 【引数】 left_motor: 左車輪駆動用モーター。
        # 【引数】 color_sensor: 路面のHSV値・反射光値を読むカラーセンサー。
        # 【引数】 sonar_sensor: 前方障害物までの距離を読む超音波センサー。
        # 【引数】 gyro_sensor: 機体の旋回角・角速度を読むジャイロセンサー。
        with self._pending_lock:
            cfg = self._pending_cap_cfg
            self._pending_cap_cfg = None
        if cfg is not None:
            self._open_cap(*cfg)

        ret, frame = self.cap.read()

        if frame is None:
            cv2.waitKey(1)
            return
        t_cap = time.time()  # t_capへ、この処理で使用する設定値または計算結果を保存する。
        self.frame_id += 1

        if self.target_interested == TargetInterested.QRCODE:
            # QR認識精度を保つため元の高解像度画像を使用する
            img_orig = frame.copy()
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            with self._frame_lock:
                self._latest_gray = img_gray

            # 一定時間を過ぎた古い認識文字列を無効にする
            with self._result_lock:
                if self._detected_text and (time.time() - self._last_decode_time) > TEXT_EXPIRY_SEC:
                    self._detected_text = ""
                qr_text   = self._detected_text
                corners   = self._detected_corners
                detecting = self._is_detecting

            # 認識対象外の領域を灰色で覆い、処理範囲を分かりやすく表示する
            gray3 = cv2.cvtColor(cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            img_orig[:, :CROP_X1]  = gray3[:, :CROP_X1]
            img_orig[:, CROP_X2:]  = gray3[:, CROP_X2:]

            # 検出したQRコードの外接矩形を画像へ描画する
            if corners is not None:
                self.target_insight = True
                color = (0, 255, 0) if qr_text else (0, 200, 255)
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(img_orig, [pts], isClosed=True, color=color, thickness=LINE_THICKNESS)
            else:
                self.target_insight = False

        elif self.target_interested == TargetInterested.BOTTLE:
            # ボトル認識は処理負荷を抑えるため縮小フレームで行う
            # 縦横比を崩さず中央を16:9へ切り出し、320×180へ縮小する
            # 画角と距離の対応関係を保つため、画像を押しつぶさず切り抜く
            # 現在の状態と判定条件に応じて後続処理を分岐する。
            if self.frame_id == 1:
                print("VID first BOTTLE frame shape=%s" % (frame.shape,))
            fh, fw = frame.shape[:2]
            crop_h = int(fw * 9 / 16)  # crop_hへ、この処理で使用する設定値または計算結果を保存する。
            y0 = (fh - crop_h) // 2  # y0へ、この処理で使用する設定値または計算結果を保存する。
            frame_169 = frame[y0:y0 + crop_h, :]
            img_orig = cv2.resize(frame_169, (FRAME_WIDTH, FRAME_HEIGHT))
            img_hsv  = cv2.cvtColor(img_orig, cv2.COLOR_BGR2HSV)
            # 色を確定済みならその色だけを追跡し、未確定なら全候補色を走査する
            if self._bottle_lock_color is not None:
                candidates = [self._bottle_lock_color]
            else:
                candidates = [BottleColor.RED, BottleColor.BLUE,
                              BottleColor.YELLOW, BottleColor.BLACK]

            best = None  # bestへ、この処理で使用する設定値または計算結果を保存する。
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
                    # 黒帯とコース線を、輪郭の長さ・太さ・充填度から区別する
                    # 現在の状態と判定条件に応じて後続処理を分岐する。
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
                # ライン用のピクセル・角度変換を再利用し、帯への方位角を求める
                vxp = bcx - int(FRAME_WIDTH / 2)
                vxm = vxp * HORIZON_DISTANCE / FRAME_WIDTH
                self.bottle_theta = 180 * math.atan(vxm / AXLE_TO_HORIZON_DISTANCE) / math.pi
                in_blind = bbottom >= BOTTLE_BLIND_ROW
                # 後段の共通表示処理へ認識結果を渡す
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

        else:  # 上記条件に当てはまらない場合の処理。
            # ライン認識は処理負荷を抑えるため縮小フレームで行う
            # 縦横比を崩さず中央を16:9へ切り出し、320×180へ縮小する
            # 画角と距離の対応関係を保つため、画像を押しつぶさず切り抜く
            # 現在の状態と判定条件に応じて後続処理を分岐する。
            if self.frame_id == 1:
                print("VID first LINE frame shape=%s" % (frame.shape,))
            fh, fw = frame.shape[:2]
            crop_h = int(fw * 9 / 16)  # crop_hへ、この処理で使用する設定値または計算結果を保存する。
            y0 = (fh - crop_h) // 2  # y0へ、この処理で使用する設定値または計算結果を保存する。
            frame_169 = frame[y0:y0 + crop_h, :]
            img_orig = cv2.resize(frame_169, (FRAME_WIDTH, FRAME_HEIGHT))
            img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)

            # 二値化する画像範囲を切り出す
            img_gray_part = img_gray[CROP_U_LIMIT:CROP_D_LIMIT, CROP_L_LIMIT:CROP_R_LIMIT]
            # 画像を二値化する
            img_bin_part = cv2.inRange(img_gray_part, self.gsmin, self.gsmax)
            # 結果格納用の空行列を準備する
            img_bin = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
            # 処理結果を出力先へコピーする
            img_bin[CROP_U_LIMIT:CROP_U_LIMIT+img_bin_part.shape[0], CROP_L_LIMIT:CROP_L_LIMIT+img_bin_part.shape[1]] = img_bin_part
            # モルフォロジー処理でノイズを除去する
            img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernel)
            # 後段で描画できるよう二値画像をBGR形式へ変換する
            #img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

            # 関心領域だけを処理対象にする
            x, y, w, h = self.roi
            img_roi = img_bin_mor[y:y+h, x:x+w]
            # 座標オフセットを考慮して関心領域内の輪郭を検出する
            contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
            # 面積が最大の輪郭を特定する
            if len(contours) >= 1:
                i_target = 0
                if len(contours) >= 2:
                    cnt_idx = np.empty((0,2), int)
                    for i, cnt in enumerate(contours):
                        area = cv2.contourArea(cnt)
                        cnt_idx = np.append(cnt_idx, np.array([[i, int(area)]]), axis=0)
                    # 輪郭番号を面積の大きい順に並べる
                    cnt_idx = cnt_idx[np.argsort(cnt_idx[:, 1])[::-1]]
                    # 後続の制御に必要な値を計算する
                    # 後続処理で使用する状態または設定値を更新する
                    x1, y1, w1, h1 = cv2.boundingRect(contours[cnt_idx[0,0]])
                    x2, y2, w2, h2 = cv2.boundingRect(contours[cnt_idx[1,0]])
                    x, y, w, h = self.roi
                    # 2番目の輪郭が関心領域下端へ接し、十分な大きさかを確認する
                    # 現在の状態と判定条件に応じて後続処理を分岐する。
                    if y2 + h2 >= h and 2*cnt_idx[1,1] >= cnt_idx[0,1]:
                        if self.trace_side == TraceSide.LEFT:
                            # 左側の矩形候補を探す
                            if x1 <= x2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        elif self.trace_side == TraceSide.RIGHT:
                            # 右側の矩形候補を探す
                            if x1 + w1 >= x2 + w2:
                                i_target = cnt_idx[0,0]
                            else:
                                i_target = cnt_idx[1,0]
                        else:  # 上記条件に当てはまらない場合の処理。
                            i_target = cnt_idx[0,0]
                    else: # 2番目の輪郭が関心領域下端へ接し、十分な大きさかを確認する
                        i_target = cnt_idx[0,0]

                # 確認表示のため、対象輪郭を元画像へ描画する
                img_orig = cv2.polylines(img_orig, [contours[i_target]], 0, (0,255,0), LINE_THICKNESS)

                # 後続の制御に必要な値を計算する
                # 後続処理で使用する状態または設定値を更新する
                x, y, w, h = cv2.boundingRect(contours[i_target])
                # 関心領域が画像範囲内に収まるよう調整する
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
                # 次フレームで使用する新しい関心領域を設定する
                self.roi = (x, y, w, h)
            
                # 追従目標位置の計算を準備する
                img_cnt = np.zeros_like(img_orig)
                img_cnt = cv2.drawContours(img_cnt, [contours[i_target]], 0, (0,255,0), 1)
                img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)

                # 帯状走査によりライン位置を測定し、現在の曲率を先行補正する
                # 機体に最も近い走査行からライン幅を求め、分岐・合流判定に使用する
                b_edges = np.flatnonzero(img_cnt_gray[SCAN_V_POS])
                if len(b_edges) >= 2:
                    self.range_of_edges = int(b_edges[-1] - b_edges[0])
                elif len(b_edges) == 1:
                    self.range_of_edges = 1
                else:
                    self.range_of_edges = 0

                # 接線状態でない有効行だけからライン中心を収集し、誤った操舵と傾き推定を防ぐ
                # samplesへ後続処理で使用する計算結果を保存する。
                samples = []  # samplesへ、この処理で使用する設定値または計算結果を保存する。
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
                    near_row, cx_raw = samples[0]  # near_row, cx_rawへ、この処理で使用する設定値または計算結果を保存する。

                    # 局所的な曲率仮定を保つ範囲で、最も遠い有効行を選ぶ
                    # far_row, cx_farへ後続処理で使用する計算結果を保存する。
                    # 分岐先のラインを誤って参照しないよう、探索範囲を局所領域に限定する
                    far_row, cx_far = near_row, cx_raw
                    for r, c in samples:
                        if (near_row - r) <= CURV_BAND_ROWS:
                            far_row, cx_far = r, c
                        else:
                            break

                    bias = 0
                    sep = near_row - far_row
                    self.band_sep = sep  # self.band_sepへ、この処理で使用する設定値または計算結果を保存する。

                    # 近側と遠側のライン中心差から画像上のライン傾きを求める
                    # 認識状況を確認できるよう画像へ線を描画する。
                    # 傾きが急な場合は機体がカーブ内側をショートカットしている可能性がある
                    # 認識状況を確認できるよう画像へ線を描画する。
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

                    # 調整用ログへ補正前後の位置、補正量、推定条件を記録する
                    # 前の条件に該当しなかった場合の処理へ分岐する。
                    #if bias != 0:
                    #    print("%+06d CRV fid=%06d cx_raw=%03d bias=%+03d cx=%03d n=%02d sep=%02d" % (
                    #        plotter.get_distance() if plotter is not None else 0,
                    # 前の条件に該当しなかった場合の処理へ分岐する。
                else:
                    # 帯全体が接線または空なら目標位置を使わず、現在方位を維持する
                    self.target_insight = False

            else: # len(contours) == 0
                self._blind_frames += 1
                self.range_of_edges = 0
                self.line_tilt = 0.0
                self.band_sep = 0
                if self._blind_frames <= ROI_HOLD_FRAMES:
                    # 短時間の見失いでは直前の関心領域を段階的に広げ、実際の移動先で再捕捉する
                    # 後続処理で使用する状態または設定値を更新する
                    # x, y, w, hへ後続処理で使用する計算結果を保存する。
                    x, y, w, h = self.roi
                    x -= ROI_BOUNDARY; y -= ROI_BOUNDARY
                    w += 2*ROI_BOUNDARY; h += 2*ROI_BOUNDARY
                    if x < CROP_L_LIMIT: x = CROP_L_LIMIT
                    if y < CROP_U_LIMIT: y = CROP_U_LIMIT
                    if x + w > CROP_R_LIMIT: w = CROP_R_LIMIT - x
                    if y + h > CROP_D_LIMIT: h = CROP_D_LIMIT - y
                    self.roi = (x, y, w, h)
                else:
                    # ライン消失が続いた場合は画像全体の探索へ戻す
                    self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
                # 再捕捉まで現在の操舵目標を維持し、関連する位置情報の整合性も保つ
                # self.cxへ後続処理で使用する計算結果を保存する。
                self.cx = self.mx
                self.cy = SCAN_V_POS
                self.target_insight = False
            
            # 確認表示のため、関心領域を元画像へ描画する
            x, y, w, h = self.roi
            if self.target_insight:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
            else:
                cv2.rectangle(img_orig, (x,y), (x+w,y+h), (0,0,255), LINE_THICKNESS)
            # 画像上に追従目標を描画する
            cv2.circle(img_orig, (self.mx, SCAN_V_POS), CIRCLE_RADIUS, (0,0,255), -1)
            # 画像中心と追従目標のずれをピクセル単位で計算する
            vxp = self.mx - int(FRAME_WIDTH/2)
            # ピクセル単位のずれをミリメートルへ変換する
            # HORIZON_DISTANCEはカメラ内で最も手前に見える地面上の水平線の長さを表す
            vxm = vxp * HORIZON_DISTANCE / FRAME_WIDTH
            # Z軸回りの旋回量をラジアンで計算する
            # HORIZON_DISTANCEはカメラ内で最も手前に見える地面上の水平線の長さを表す
            self.theta = 180 * math.atan(vxm / AXLE_TO_HORIZON_DISTANCE) / math.pi
            # 操舵角をフレーム番号・撮影時刻とともに公開し、カメラ追従処理から利用する
            dist = plotter.get_distance() if plotter is not None else 0
            with self._theta_lock:
                self._theta_stamped = (self.theta, self.frame_id, t_cap, dist)
            #print(
            # 送信・表示または認識に適した解像度へ画像を縮小する。
            #        int(self.target_insight), (time.time() - t_cap) * 1000))

        # ここから下はすべての認識対象に共通する処理
        # 処理画像を送信・表示用の解像度へ直接縮小する
        # 送信・表示または認識に適した解像度へ画像を縮小する。
        img_mon = cv2.resize(img_orig, (OUT_FRAME_WIDTH, OUT_FRAME_HEIGHT))

        # 画像と縦結合できるよう、モニター画像と同じ幅の文字表示領域を準備する
        img_text = np.zeros((OUT_FRAME_HEIGHT, OUT_FRAME_WIDTH, 3), np.uint8)
        ts     = TEXT_SCALE
        f_norm = 1.8 * ts  # f_normへ、この処理で使用する設定値または計算結果を保存する。
        f_fid  = 2.6 * ts  # f_fidへ、この処理で使用する設定値または計算結果を保存する。
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
        # 画像と縦結合できるよう、モニター画像と同じ幅の文字表示領域を準備する
        img_comm = cv2.vconcat([img_mon, img_text])
        # 処理済み画像を送信し、モニターへ表示する
        cv2.imshow("video monitor", img_comm)
        cv2.waitKey(1)  # この行で指定する値の用途を示す。
        return
        
    # 【日本語解説】 ラインまたは対象物へ向くための最新の操舵角を返す。
    def get_theta(self) -> float:
        return self.theta

    # 【日本語解説】 操舵角と、その値を取得した時刻を組にして返す。
    def get_theta_stamped(self):
        with self._theta_lock:
            return self._theta_stamped  # この行で指定する値の用途を示す。

    # 【日本語解説】 検出したラインの画像上の傾きを返す。
    def get_line_tilt(self) -> float:
        return self.line_tilt

    # 【日本語解説】 近側と遠側の走査帯で検出したライン位置の差を返す。
    def get_band_sep(self) -> int:
        return self.band_sep

    # 【日本語解説】 画像中で検出したライン両端の広がりを返す。
    def get_range_of_edges(self) -> int:
        return self.range_of_edges

    # 【日本語解説】 HSV画像から指定色のボトル候補画素だけを抽出した二値マスクを作る。
    def _bottle_mask(self, img_hsv, color):
        # 【引数】 img_hsv: 色抽出に使用するHSV形式の画像。
        # 【引数】 color: 検出・照合の対象とする色。
        mask = None
        for lo, hi in BOTTLE_HSV[color]:
            m = cv2.inRange(img_hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        return mask

    # 【日本語解説】 ボトル検出結果と検出時刻を組にして返す。
    def get_bottle_stamped(self):
        with self._bottle_lock:
            return self._bottle_stamped  # この行で指定する値の用途を示す。

    # 【日本語解説】 現在の検出対象として設定されたボトル色を返す。
    def get_bottle_color(self) -> 'BottleColor':
        with self._bottle_lock:
            return self._bottle_stamped[1]

    # 【日本語解説】 画像処理が探索するボトル色を設定する。
    def set_bottle_color(self, color) -> None:
        # 【引数】 color: 検出・照合の対象とする色。
        self._bottle_lock_color = color

    # 【日本語解説】 最後に正常認識できたQRコード文字列を返す。
    def get_QR_text(self) -> str:
        if self.target_interested == TargetInterested.QRCODE:
            with self._result_lock:
                return self._detected_text
        else:
            # 認識文字列はモード再設定または有効期限切れまで保持する
            # 計算または判定した結果を呼び出し元へ返す。
            return self._detected_text

    # 【日本語解説】 ライン抽出に使うグレースケール値の下限と上限を設定する。
    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        # 【引数】 gs_min: ラインとして抽出するグレースケール値の下限。
        # 【引数】 gs_max: ラインとして抽出するグレースケール値の上限。
        self.gsmin = gs_min
        self.gsmax = gs_max
        return

    # 【日本語解説】 ラインの左右どちら側を基準に追従するかを設定する。
    def set_trace_side(self, trace_side: TraceSide) -> None:
        # 【引数】 trace_side: ラインの通常側・反対側のどちらの端を追従するか。
        self.trace_side = trace_side
        return

    # 【日本語解説】 計算負荷を抑えるため、現在認識すべき対象へ画像処理モードを切り替える。
    def set_target_interested(self, target_interested: TargetInterested) -> None:
        # 【引数】 target_interested: 画像処理が現在認識すべき対象。
        self.target_interested = target_interested

        # 対象に適した映像形式への変更を要求し、実際の再オープンはキャプチャースレッドで行う
        # cfgへ後続処理で使用する計算結果を保存する。
        cfg = _CAP_CONFIG.get(target_interested)
        if cfg is not None and cfg != self._cap_cfg:
            with self._pending_lock:
                self._pending_cap_cfg = cfg

        if self.target_interested == TargetInterested.QRCODE and not hasattr(self, "_detection_thread"):
            self._detected_text    = ""  # self._detected_textへ、この処理で使用する設定値または計算結果を保存する。
            # QR検出スレッドを開始する
            self._detection_thread = threading.Thread(target=self._detection_worker, daemon=True)
            self._detection_thread.start()
        else:
            # QR認識モード以外では検出スレッドを停止する
            if hasattr(self, "_detection_thread"):
                self._detection_thread.join(timeout=1.0)
                del self._detection_thread
        return

    # 【日本語解説】 現在の探索対象がカメラ視野内で検出されているかを返す。
    def is_target_insight(self) -> bool:
        return self.target_insight
