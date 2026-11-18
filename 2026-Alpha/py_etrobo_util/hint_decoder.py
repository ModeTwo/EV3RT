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
    # 大会指定の4文字のキー(英数字)以外を復号処理へ渡さない。2026-11-18(devSS): 英字も受け付ける(大文字・小文字はそのまま)。
    if (not isinstance(decryption_key, str)
            or len(decryption_key) != 4
            or not decryption_key.isascii()
            or not decryption_key.isalnum()):
        raise ValueError("A four-character (letters or digits) decryption key is required")
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
    # 2026-11-18(devSS、本番の試走): キー T111 の回に復号できなかった(小文字で打った疑い)。
    # 入力どおりで解けなければ、大文字・小文字を替えたキーも試す(違うキーで読める文字が出ることはまずない)。
    tried = []
    for candidate in (decryption_key, decryption_key.upper(), decryption_key.lower(), decryption_key.swapcase()):
        if candidate in tried:
            continue
        tried.append(candidate)
        key = PBKDF2(
            candidate.encode("ascii"),
            salt,
            dkLen=16,
            count=10000,
            hmac_hash_module=SHA256,
        )
        try:
            plaintext = unpad(AES.new(key, AES.MODE_ECB).decrypt(ciphertext), 16)
            text = plaintext.decode("utf-8")
        except (ValueError, UnicodeDecodeError):
            continue
        # 入力どおりのキーは従来どおり。替えたキーは読める文字のときだけ採用する。
        if candidate == decryption_key or (text.strip() and text.strip().isprintable()):
            if candidate != decryption_key:
                print(" -- hint2: decrypted with key %r (entered %r)" % (candidate, decryption_key), flush=True)
            return text
    raise ValueError("Hint 2 decryption failed (tried keys %s)" % ", ".join(repr(k) for k in tried))
