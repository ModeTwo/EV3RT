# 【日本語解説】 Raspberry PiからSPIKEとWebカメラを連携させ、ETロボコン2026の走行・画像認識を制御する。
# 【日本語解説】 行動木の各update()は短時間で1周期だけ処理し、完了まではRUNNINGを返す。
import re
import base64
from enum import Enum
from typing import Optional, Tuple

from Crypto.Cipher import AES
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Util.Padding import unpad
from Crypto.Hash import SHA256


# 【日本語解説】 QRコードから取得した競技ヒントの形式を表す列挙型。
class HintType(Enum):
    HINT1 = "hint1"  # HINT1へ、この処理で使用する設定値または計算結果を保存する。
    HINT2 = "hint2"  # HINT2へ、この処理で使用する設定値または計算結果を保存する。
    UNKNOWN = "unknown"


# 【日本語解説】 QR文字列の形式判定と、OpenSSL互換AESデータの復号を担当するクラス。
class Hint:
    PASSWORD = "0000"

    HINT1_RE = re.compile(
        r'''
        ^                # start of string
        [1-5]{2}         # exactly two digits, each 1‑5   → first number
        ,                # literal comma
        [1-5]{2}         # exactly two digits, each 1‑5   → second number
        $                # end of string
        ''',
        re.VERBOSE,
    )

    BASE64_RE = re.compile(
        r'''
        ^                                 # start of string
        (?:                               # repeat groups of 4 characters …
            [A-Za-z0-9+/]{4}              #   4 "real" Base‑64 chars
        )*                                #
        (?:                               # last quantum may be padded:
            [A-Za-z0-9+/]{2}==            #   two data chars + "=="
          | [A-Za-z0-9+/]{3}=             #   three data chars + "="
        )?                                #
        $                                 # end of string
        ''',
        re.VERBOSE,
    )

    # 【日本語解説】 Hintの設定値と実行中に保持する状態を初期化する。
    def __init__(self, raw: str):
        # 【引数】 raw: QRコードから取得した未加工のヒント文字列。
        self.raw = raw
        self.type = self._classify(raw)

    @classmethod
    # 【日本語解説】 文字列形式を正規表現で調べ、該当するヒント種別を返す。
    def _classify(cls, s: str) -> HintType:
        # 【引数】 s: ヒント形式を判定する入力文字列。
        if cls.HINT1_RE.match(s):
            return HintType.HINT1
        if s and cls.BASE64_RE.match(s):
            return HintType.HINT2
        return HintType.UNKNOWN

    @property
    # 【日本語解説】 hint1の条件を満たしているかを真偽値で返す。
    def is_hint1(self) -> bool:
        return self.type is HintType.HINT1

    @property
    # 【日本語解説】 hint2の条件を満たしているかを真偽値で返す。
    def is_hint2(self) -> bool:
        return self.type is HintType.HINT2

    # 【日本語解説】 Base64を展開し、PBKDF2で導出した鍵を使ってAES暗号文を復号する。
    def _decrypt(self, password: Optional[str] = None) -> str:
        # 【引数】 password: 暗号化ヒントの復号パスワード。Noneなら既定値を使う。
        """OpenSSL互換のSalted__ヘッダー、PBKDF2、AES-ECBで暗号化されたデータを復号する。パスワード省略時はクラス既定値を使う。"""
        pw = self.PASSWORD if password is None else password

        # デコード前に空白と改行を安全に取り除く
        encoded_bytes = b"".join(self.raw.encode("utf-8").split())
        data = base64.b64decode(encoded_bytes)

        if data[:8] != b"Salted__":
            raise ValueError("Missing OpenSSL salt header")

        salt = data[8:16]
        ciphertext = data[16:]
        password_bytes = b"".join(pw.encode("utf-8").split())

        # OpenSSLのPBKDF2既定値（反復10000回、SHA-256）を使用する
        key = PBKDF2(
            password_bytes,
            salt,
            dkLen=16,
            count=10000,
            hmac_hash_module=SHA256,
        )

        cipher = AES.new(key, AES.MODE_ECB)
        plaintext = unpad(cipher.decrypt(ciphertext), 16)
        return plaintext.decode("utf-8")

    # 【日本語解説】 ヒント種別を返し、暗号形式の場合だけ指定パスワードで復号する。
    def resolve(self, password: Optional[str] = None) -> Tuple[HintType, str]:
        # 【引数】 password: 暗号化ヒントの復号パスワード。Noneなら既定値を使う。
        """ヒント種別と内容を返す。HINT1は元文字列、HINT2は復号結果、UNKNOWNは未加工文字列を返す。"""
        if self.type is HintType.HINT2:
            return self.type, self._decrypt(password)
        return self.type, self.raw

    # 【日本語解説】 デバッグ用にHintの内容が分かる文字列表現を返す。
    def __repr__(self) -> str:
        return f"Hint({self.raw!r}, type={self.type.value})"