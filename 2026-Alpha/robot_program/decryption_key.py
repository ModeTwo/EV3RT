"""Startup input for the four-digit Hint 2 decryption key."""


def read_decryption_key(input_fn=input, output_fn=print):
    # 実機・カメラ・20ms周期を開始する前に、入力と確認を完了させる。
    while True:
        value = input_fn("Enter the four-digit decryption key: ").strip()
        if len(value) != 4 or not value.isascii() or not value.isdigit():
            output_fn("The key must contain exactly four ASCII digits.")
            continue

        confirmation = input_fn("Use this decryption key? (y/n): ").strip().lower()
        if confirmation == "y":
            return value
        if confirmation != "n":
            output_fn("Enter y or n.")

