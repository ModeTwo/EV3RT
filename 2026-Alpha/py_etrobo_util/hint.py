import re
from enum import Enum
from typing import Optional, Tuple

# 通信PC用フォルダを配備しない場合も、走行体単独で起動できるようにする。
from .hint_decoder import decode_hint2


class HintType(Enum):
    HINT1 = "hint1"      # e.g. "12,34" — two pairs of digits, each 1–5
    HINT2 = "hint2"      # Base‑64 / OpenSSL "Salted__" payload
    UNKNOWN = "unknown"


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

    def __init__(self, raw: str):
        self.raw = raw
        self.type = self._classify(raw)

    @classmethod
    def _classify(cls, s: str) -> HintType:
        if cls.HINT1_RE.match(s):
            return HintType.HINT1
        if s and cls.BASE64_RE.match(s):
            return HintType.HINT2
        return HintType.UNKNOWN

    @property
    def is_hint1(self) -> bool:
        return self.type is HintType.HINT1

    @property
    def is_hint2(self) -> bool:
        return self.type is HintType.HINT2

    def _decrypt(self, password: Optional[str] = None) -> str:
        """Decrypt an OpenSSL `Salted__` + PBKDF2 + AES‑ECB payload.

        Uses `password` if given, otherwise falls back to `self.PASSWORD`.
        """
        return decode_hint2(self.raw, self.PASSWORD if password is None else password)

    def resolve(self, password: Optional[str] = None) -> Tuple[HintType, str]:
        """
        Return the hint type and its payload:
          • HINT1 → the raw string as‑is
          • HINT2 → the decrypted plaintext (using `password` if provided,
                    otherwise the class default)
          • UNKNOWN → the raw string (nothing to decode)
        """
        if self.type is HintType.HINT2:
            return self.type, self._decrypt(password)
        return self.type, self.raw

    def __repr__(self) -> str:
        return f"Hint({self.raw!r}, type={self.type.value})"
