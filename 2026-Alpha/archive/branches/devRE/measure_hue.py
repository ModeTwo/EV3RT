#青色のh(h,s,vのh)を計測する
#青と白の境界に置く

import time
from etrobo_python import ETRobo, Hub, ColorSensor

def main():
    # ETRobo 初期化
    robo = ETRobo()
    hub = Hub()
    color_sensor = ColorSensor()

    print("=== Hue 計測開始 ===")
    print("青ラインの上にセンサーを置いてください")
    print("Ctrl+C で終了します\n")

    try:
        while True:
            # HSV を取得
            h, s, v = color_sensor.get_raw_color_hsv()

            # Hue を表示
            print(f"H={h}, S={s}, V={v}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n=== Hue 計測終了 ===")

if __name__ == "__main__":
    main()
