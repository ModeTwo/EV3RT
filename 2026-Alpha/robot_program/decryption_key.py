"""Startup input for the four-character Hint 2 decryption key (ASCII letters or digits)."""


def read_decryption_key(input_fn=input, output_fn=print):
    # 実機・カメラ・20ms周期を開始する前に、入力と確認を完了させる。
    while True:
        value = input_fn("Enter the four-character decryption key (letters or digits): ").strip()
        # 2026-11-18(devSS、ユーザー): 英字も受け付ける(大文字・小文字はそのまま使う)。
        if len(value) != 4 or not value.isascii() or not value.isalnum():
            output_fn("The key must contain exactly four ASCII letters or digits.")
            continue

        confirmation = input_fn("Use this decryption key? (y/n): ").strip().lower()
        if confirmation == "y":
            return value
        if confirmation != "n":
            output_fn("Enter y or n.")

