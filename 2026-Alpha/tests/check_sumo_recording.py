"""Real MJPG/CSV saves and bounded recording, without robot hardware."""
import csv
import json
from pathlib import Path
import runpy
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

BOOTSTRAP = runpy.run_path(str(Path(__file__).with_name('check_sumo_align.py')))
Video = BOOTSTRAP['Video']
from robot_program.features.sumo_align_recording import SumoAlignRecording


class RecordingTests(unittest.TestCase):
    def test_diagnostic_columns_survive_real_csv_writer(self):
        recorder = SumoAlignRecording(output_dir=self.root)
        row = self.row(1)
        row.update(track_id=7, mask_source='value', detector_duration_ms=2.5,
                   selected_frame_id=1, requested_turn_step_deg=-4.2,
                   brake_command_monotonic_sec=9.8)
        self.assertTrue(recorder.submit(self.raw, self.detected, row))
        recorder.request_stop('SUCCESS', 'test')
        recorder.close()
        actual = self.rows(recorder)[0]
        for key in ('track_id', 'mask_source', 'detector_duration_ms', 'selected_frame_id',
                    'requested_turn_step_deg', 'brake_command_monotonic_sec'):
            self.assertEqual(actual[key], str(row[key]))

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix='sumo-video-test-')
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.raw = np.full((480, 640, 3), 220, np.uint8)
        self.raw[140:200, 300:340] = 0
        self.detected = cv2.resize(self.raw[60:420], (320, 180))

    def row(self, frame, phase='DRIVE'):
        return dict(frame_id=frame, capture_monotonic_sec=10 + .2 * frame,
                    capture_session=1, requested_session=1, capture_mode='sumo_align',
                    phase=phase, turns=0, decision='no-candidate', confirmed=0,
                    candidate=0, observation_available=1, odo_mm=10 * frame, gyro_deg=0)

    def rows(self, recorder):
        with recorder.paths['csv'].open(encoding='utf-8', newline='') as stream:
            return list(csv.DictReader(stream))

    def decoded(self, path):
        capture = cv2.VideoCapture(str(path))
        sizes = []
        while True:
            ok, image = capture.read()
            if not ok:
                break
            sizes.append((image.shape[1], image.shape[0]))
        capture.release()
        return sizes

    def test_two_videos_csv_and_failure_summary_have_matching_frames(self):
        recorder = SumoAlignRecording(output_dir=self.root)
        phases = ('RECORD_START', 'DRIVE', 'SETTLE', 'OBSERVE', 'TURN', 'OBSERVE')
        for frame, phase in enumerate(phases, 1):
            self.assertTrue(recorder.submit(self.raw, self.detected, self.row(frame, phase)))
        self.assertTrue(recorder.ready.wait(3))
        recorder.request_stop('FAILURE', 'no fresh stable bottle after stopping')
        recorder.close()
        self.assertIsNone(recorder.status()['error'])
        self.assertEqual(self.decoded(recorder.paths['raw']), [(640, 480)] * 6)
        self.assertEqual(self.decoded(recorder.paths['detected']), [(320, 180)] * 6)
        rows = self.rows(recorder)
        self.assertEqual([r['phase'] for r in rows], list(phases))
        self.assertEqual([int(r['frame_id']) for r in rows], list(range(1, 7)))
        self.assertAlmostEqual(float(rows[-1]['elapsed_sec']), 1.0)
        summary = json.loads(recorder.paths['summary'].read_text(encoding='utf-8'))
        self.assertEqual(summary['result'], 'FAILURE')
        self.assertEqual(summary['frames_written'], 6)

    def test_duplicate_frames_are_not_saved_twice(self):
        recorder = SumoAlignRecording(output_dir=self.root)
        self.assertTrue(recorder.submit(self.raw, self.detected, self.row(1)))
        self.assertFalse(recorder.submit(self.raw, self.detected, self.row(1)))
        recorder.request_stop('SUCCESS', 'aligned')
        recorder.close()
        self.assertEqual(len(self.rows(recorder)), 1)

    def test_codec_failure_is_reported_and_files_are_closed(self):
        with patch('robot_program.features.sumo_align_recording.cv2.VideoWriter') as factory:
            factory.return_value.isOpened.return_value = False
            recorder = SumoAlignRecording(output_dir=self.root)
            recorder.submit(self.raw, self.detected, self.row(1))
            self.assertTrue(recorder.finished.wait(3))
            recorder.close()
        self.assertIn('Cannot open', recorder.status()['error'])
        self.assertTrue(recorder.paths['csv'].is_file())
        self.assertTrue(recorder.paths['summary'].is_file())

    def test_slow_encoder_drops_frames_instead_of_blocking_capture(self):
        entered, release = threading.Event(), threading.Event()

        class SlowWriter:
            def __init__(self, *args):
                pass

            def isOpened(self):
                return True

            def write(self, frame):
                entered.set()
                release.wait(3)

            def release(self):
                pass

        with patch('robot_program.features.sumo_align_recording.cv2.VideoWriter', SlowWriter):
            recorder = SumoAlignRecording(output_dir=self.root, queue_size=1)
            recorder.submit(self.raw, self.detected, self.row(1))
            self.assertTrue(entered.wait(3))
            self.assertTrue(recorder.submit(self.raw, self.detected, self.row(2)))
            self.assertFalse(recorder.submit(self.raw, self.detected, self.row(3)))
            release.set()
            recorder.request_stop('SUCCESS', 'aligned')
            recorder.close()
        self.assertEqual(len(self.rows(recorder)), 2)
        self.assertEqual(recorder.status()['dropped'], 1)

    def test_video_records_across_session_resets_without_preview(self):
        with patch.object(Video, '_open_cap'):
            video = Video(preview_enabled=False)
        self.addCleanup(video.close)
        phase = dict(value='DRIVE')
        video.cap = Mock()
        video.cap.read.side_effect = lambda: (True, self.raw.copy())
        video.begin_sumo_alignment_recording(lambda: dict(phase=phase['value'], turns=0,
            requested_session=video._vision_sessions.capture_token()[0], target_cx=160), output_dir=self.root)
        video.begin_sumo_alignment_read(gyro_reader=lambda: 2, distance_reader=lambda: 123)
        with patch.object(Video, '_open_cap'):
            for i in range(4):
                if i == 2:
                    phase['value'] = 'OBSERVE'
                    video.begin_sumo_alignment_read(gyro_reader=lambda: 2, distance_reader=lambda: 123)
                video.process(None, None, None, None, None, None, None, None)
        video.end_sumo_alignment_recording('FAILURE', 'test detection timeout')
        video._sumo_recording.close()
        rows = self.rows(video._sumo_recording)
        self.assertEqual(len(rows), 4)
        self.assertEqual([r['phase'] for r in rows], ['DRIVE', 'DRIVE', 'OBSERVE', 'OBSERVE'])
        self.assertEqual(len({r['capture_session'] for r in rows}), 2)
        self.assertTrue(all(r['observation_available'] == '1' for r in rows))
        self.assertTrue(all(r['track_id'] for r in rows))
        self.assertTrue(all(float(r['detector_duration_ms']) >= 0 for r in rows))
        self.assertEqual(self.decoded(video._sumo_recording.paths['raw']), [(640, 480)] * 4)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + BOOTSTRAP['TEST_ARGS'])
