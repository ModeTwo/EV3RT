"""Robot-local Hint 2 decryption without wireless service dependencies."""

import base64



def decode_hint2(raw_text, decryption_key):
    # 相撲など復号しない工程では暗号ライブラリを必須にしない。
    # 復号仕様は従来の通信共通処理と同じまま、依存関係だけを分離する。
    try:
        from Crypto.Cipher import AES
        from Crypto.Hash import SHA256
        from Crypto.Protocol.KDF import PBKDF2
        from Crypto.Util.Padding import unpad
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "Hint 2 decryption requires pycryptodome in the active Python environment"
        ) from error
    # 大会指定の4桁キー以外を復号処理へ渡さない。
    if (not isinstance(decryption_key, str)
            or len(decryption_key) != 4
            or not decryption_key.isascii()
            or not decryption_key.isdigit()):
        raise ValueError("A four-digit decryption key is required")
    if not isinstance(raw_text, str) or not raw_text:
        raise ValueError("Hint 2 raw text is required")

    try:
        encoded = b"".join(raw_text.encode("utf-8").split())
        encrypted = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as error:
        raise ValueError("Hint 2 is not valid Base64") from error
    if encrypted[:8] != b"Salted__" or len(encrypted) < 32:
        raise ValueError("Hint 2 has no OpenSSL salt header")

    salt = encrypted[8:16]
    ciphertext = encrypted[16:]
    key = PBKDF2(
        decryption_key.encode("ascii"),
        salt,
        dkLen=16,
        count=10000,
        hmac_hash_module=SHA256,
    )
    try:
        plaintext = unpad(AES.new(key, AES.MODE_ECB).decrypt(ciphertext), 16)
        return plaintext.decode("utf-8")
    except (ValueError, UnicodeDecodeError) as error:
        raise ValueError("Hint 2 decryption failed") from error
