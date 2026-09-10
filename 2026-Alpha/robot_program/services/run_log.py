"""画面出力を実行ごとのUTF-8ログへ複製する。走行制御から独立した入口用処理。"""

from datetime import datetime, timezone
import io
import os
from pathlib import Path
import re
import sys
import threading
import time
import traceback
from uuid import uuid4

ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


class _LogFile:
    def __init__(self, stream, error_stream):
        self.stream = stream
        self.error_stream = error_stream
        self.started = time.monotonic()
        self.line_start = True
        self.failed = False
        self.lock = threading.RLock()

    def write(self, text, channel):
        """端末の自動折返しを含まない、元の1行を保存する。"""
        with self.lock:
            if self.failed:
                return
            try:
                for piece in ANSI_ESCAPE.sub('', text).splitlines(keepends=True):
                    if self.line_start:
                        utc = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
                        self.stream.write(f'{utc} +{time.monotonic()-self.started:.3f}s [{channel}] ')
                    self.stream.write(piece)
                    self.line_start = piece.endswith(('\n', '\r'))
                if self.line_start:
                    self.stream.flush()
            except (OSError, ValueError) as error:
                # 容量不足等で走行制御へ例外を伝播させない。保存停止を一度知らせる。
                self.failed = True
                try:
                    self.error_stream.write(f'\n[run log] File logging stopped: {error}\n')
                    self.error_stream.flush()
                except (OSError, ValueError):
                    pass


class _Tee:
    def __init__(self, console, log, channel):
        self.console, self.log, self.channel = console, log, channel

    def write(self, text):
        # 保存を先に行い、端末切断時にも可能な範囲で記録を残す。
        self.log.write(text, self.channel)
        return self.console.write(text)

    def flush(self):
        self.console.flush()

    def __getattr__(self, name):
        return getattr(self.console, name)


def run_with_log(main, log_dir=None):
    """main()を実行し、正常/例外/Ctrl+Cを記録。終了コード・例外は変更しない。

    通常はalpha.py末尾からだけ呼ぶ。mainを直接呼ぶテストには副作用を加えない。
    保存先は2026-Alpha/run_logs。実行ディレクトリやPC名に依存しない。
    """
    folder = Path(log_dir) if log_dir is not None else Path(__file__).resolve().parents[2] / 'run_logs'
    folder.mkdir(parents=True, exist_ok=True)
    name = datetime.now(timezone.utc).strftime('run_%Y%m%dT%H%M%S_%fZ')
    path = folder / f'{name}_{os.getpid()}_{uuid4().hex[:8]}.log'
    # ファイルが作れない場合はmainより前に失敗し、未記録のまま走行を開始しない。
    stream = path.open('x', encoding='utf-8', newline='\n')
    old_out, old_err = sys.stdout, sys.stderr
    log = _LogFile(stream, old_err)
    sys.stdout, sys.stderr = _Tee(old_out, log, 'stdout'), _Tee(old_err, log, 'stderr')
    outcome = 'not completed'
    try:
        print(f'[run log] Recording: {path}', flush=True)
        log.write(f'argv={sys.argv!r}\n', 'session')
        result = main()
        outcome = f'returned {result!r}'
        return result
    except BaseException as error:
        outcome = f'{type(error).__name__}: {error}'
        if not isinstance(error, (SystemExit, KeyboardInterrupt)):
            trace = io.StringIO()
            traceback.print_exc(file=trace)
            log.write(trace.getvalue(), 'exception')
        raise
    finally:
        log.write(f'\nEND {outcome}\n', 'session')
        sys.stdout, sys.stderr = old_out, old_err
        try:
            stream.close()
        except OSError as error:
            log.failed = True
            try:
                old_err.write(f'[run log] Close failed: {error}\n')
            except (OSError, ValueError):
                pass
        try:
            print(f'[run log] {"INCOMPLETE" if log.failed else "Saved"}: {path}', file=old_err, flush=True)
        except (OSError, ValueError):
            pass
