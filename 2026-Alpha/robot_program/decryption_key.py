"""Startup input for the four-character Hint 2 decryption key (letters, digits or symbols)."""


def read_decryption_key(input_fn=input, output_fn=print):
    # 実機・カメラ・20ms周期を開始する前に、入力と確認を完了させる。
    while True:
        value = input_fn("Enter the four-character decryption key: ").strip()
        # 2026-11-18(devSS、ユーザー): 英字も受け付ける(大文字・小文字はそのまま使う)。
        # 2026-11-19(devSS、ユーザー: 形式は4桁なら何でもありかも): 記号なども受け付ける。前後の空白は打ち間違いとして除く。
        if len(value) != 4 or not value.isprintable():
            output_fn("The key must be exactly four characters (got %d: %r)." % (len(value), value))
            continue

        confirmation = input_fn("Use this decryption key? (y/n): ").strip().lower()
        if confirmation == "y":
            # 2026-11-18: 復号できなかったときに打ったキーが分かるようにログへ出す。
            output_fn(" -- decryption key entered: %r" % value)
            return value
        if confirmation != "n":
            output_fn("Enter y or n.")

