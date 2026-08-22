import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2

cap = cv2.VideoCapture(0)
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("camera", frame)

    key = cv2.waitKey(1)

    # sキーで1枚だけ撮影して保存
    if key == ord('s'):
        filename = f"capture_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"{filename} を保存しました")
        count += 1

    # qキーで終了
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
