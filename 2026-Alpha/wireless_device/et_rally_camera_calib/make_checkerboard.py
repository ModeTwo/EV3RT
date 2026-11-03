"""2026-11-02: カメラ較正用のチェッカーボード(A4横、原寸)のPDFを作る。
マス25mm × 横10・縦7(内側の角 9×6)。縦横の数を偶数・奇数にして、上下左右の取り違えが起きないようにする。
下に100mmの確認用の線を入れる(印刷の縮みを定規で確かめる)。
usage: python make_checkerboard.py [出力フォルダ]
"""
import os
import sys

SQ_MM = 25.0
COLS, ROWS = 10, 7
PAGE_W_MM, PAGE_H_MM = 297.0, 210.0
PT = 72.0 / 25.4


def build_pdf_stream():
    board_w, board_h = COLS * SQ_MM, ROWS * SQ_MM
    x0 = (PAGE_W_MM - board_w) / 2.0
    y0 = (PAGE_H_MM - board_h) / 2.0 + 4.0      # 下の文字と確認用の線のぶん少し上へ
    ops = ['0 g']
    for r in range(ROWS):
        for c in range(COLS):
            if (r + c) % 2 == 0:
                ops.append('%.3f %.3f %.3f %.3f re f' % ((x0 + c * SQ_MM) * PT, (y0 + r * SQ_MM) * PT, SQ_MM * PT, SQ_MM * PT))
    # 100mmの確認用の線(両端に縦の目盛り)
    ly = y0 - 9.0
    lx = x0
    ops.append('0.3 w')
    ops.append('%.3f %.3f m %.3f %.3f l S' % (lx * PT, ly * PT, (lx + 100.0) * PT, ly * PT))
    for t in (0.0, 50.0, 100.0):
        h = 3.0 if t != 50.0 else 1.5
        ops.append('%.3f %.3f m %.3f %.3f l S' % ((lx + t) * PT, (ly - h) * PT, (lx + t) * PT, (ly + h) * PT))
    texts = [
        (lx + 104.0, ly - 1.2, 'check: this line = 100 mm'),
        (x0, ly - 8.0, 'square = %.0f mm, %d x %d squares (inner corners %d x %d). Print at 100%% (actual size), no scaling.'
         % (SQ_MM, COLS, ROWS, COLS - 1, ROWS - 1)),
    ]
    for tx, ty, s in texts:
        ops.append('BT /F1 8 Tf %.3f %.3f Td (%s) Tj ET' % (tx * PT, ty * PT, s))
    return '\n'.join(ops).encode('ascii'), (x0, y0)


def write_pdf(path):
    stream, origin = build_pdf_stream()
    objs = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        ('<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>'
         % (PAGE_W_MM * PT, PAGE_H_MM * PT)).encode('ascii'),
        b'<< /Length %d >>\nstream\n' % len(stream) + stream + b'\nendstream',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
    ]
    out = b'%PDF-1.4\n'
    offsets = []
    for i, o in enumerate(objs, 1):
        offsets.append(len(out))
        out += b'%d 0 obj\n' % i + o + b'\nendobj\n'
    xref = len(out)
    out += b'xref\n0 %d\n0000000000 65535 f \n' % (len(objs) + 1)
    for off in offsets:
        out += b'%010d 00000 n \n' % off
    out += b'trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n' % (len(objs) + 1, xref)
    with open(path, 'wb') as f:
        f.write(out)
    return origin


if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(out_dir, 'checkerboard_a4_25mm_10x7.pdf')
    x0, y0 = write_pdf(path)
    print(path, 'board origin (mm from page bottom-left): %.1f, %.1f' % (x0, y0))
