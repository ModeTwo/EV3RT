"""カメラ・走行機器を使わずに、自動ログ保存と終了動作を確認する。"""
from contextlib import redirect_stdout, redirect_stderr
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('run_log_under_test', ROOT/'robot_program/services/run_log.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RunLoggingTest(unittest.TestCase):
    def test_long_lines_stdout_stderr_and_incremental_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, err = io.StringIO(), io.StringIO()
            def main():
                print('start')
                print('profile ' + 'x'*5000)
                print('error detail', file=sys.stderr)
                current = next(Path(tmp).glob('*.log')).read_text(encoding='utf-8')
                self.assertIn('x'*5000, current)  # 終了前に保存済み、端末幅で折り返さない。
                return 7
            with redirect_stdout(out), redirect_stderr(err):
                self.assertEqual(module.run_with_log(main, tmp), 7)
                self.assertIs(sys.stdout, out)
                self.assertIs(sys.stderr, err)
            log = next(Path(tmp).glob('*.log')).read_text(encoding='utf-8')
            self.assertIn('END returned 7', log)
            self.assertIn('[stderr] error detail', log)
            self.assertIn('profile '+'x'*5000, out.getvalue())
            self.assertIn('Saved:', err.getvalue())

    def test_exception_traceback_saved_without_changing_exception(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            def main():
                print('before failure')
                raise RuntimeError('deliberate failure')
            with self.assertRaisesRegex(RuntimeError, 'deliberate failure'):
                module.run_with_log(main, tmp)
            log = next(Path(tmp).glob('*.log')).read_text(encoding='utf-8')
            self.assertIn('Traceback', log)
            self.assertIn('END RuntimeError: deliberate failure', log)

    def test_keyboard_interrupt_and_signal_exit_preserved(self):
        for error in (KeyboardInterrupt(), SystemExit(130), SystemExit(143)):
            with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                def main():
                    print('before interrupt')
                    raise error
                with self.assertRaises(type(error)) as caught:
                    module.run_with_log(main, tmp)
                self.assertIs(caught.exception, error)
                self.assertIn('END '+type(error).__name__, next(Path(tmp).glob('*.log')).read_text(encoding='utf-8'))

    def test_unique_files_and_worker_thread_output(self):
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            def main():
                worker=threading.Thread(target=lambda: print('worker message'))
                worker.start(); worker.join()
                print('\x1b[31mred text\x1b[0m')
            module.run_with_log(main, tmp)
            module.run_with_log(main, tmp)
            files=list(Path(tmp).glob('*.log'))
            self.assertEqual(len(files),2)
            for file in files:
                text=file.read_text(encoding='utf-8')
                self.assertIn('worker message',text)
                self.assertIn('red text',text)
                self.assertNotIn('\x1b',text)

    def test_file_creation_failure_does_not_start_robot(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)/'blocked'; folder.write_text('file')
            called=[]
            with self.assertRaises(FileExistsError):
                module.run_with_log(lambda: called.append(True), folder)
            self.assertEqual(called,[])

    def test_write_failure_does_not_escape_into_main(self):
        class Broken(io.StringIO):
            def write(self,text):
                raise OSError('disk full')
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()) as err:
            with patch.object(Path,'open',return_value=Broken()):
                self.assertEqual(module.run_with_log(lambda: 9,tmp),9)
            self.assertIn('File logging stopped',err.getvalue())
            self.assertIn('INCOMPLETE',err.getvalue())

    def test_entry_point_wraps_main_and_default_path_is_project_local(self):
        source=(ROOT/'alpha.py').read_text(encoding='utf-8-sig')
        self.assertIn('sys.exit(run_with_log(main))',source)
        self.assertEqual(Path(module.__file__).resolve().parents[2],ROOT)
        self.assertIn('/run_logs/',(ROOT/'.gitignore').read_text(encoding='utf-8-sig'))


if __name__=='__main__':
    unittest.main()
