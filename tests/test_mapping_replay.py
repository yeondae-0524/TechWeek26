import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import _path  # noqa: F401
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import replay_mapping as replay


OPTIONS = dict(calibration_duration=.2, stationary_wheel_speed=.001,
               max_stationary_rate=.1, max_rate=3.0, max_dt=.2)


def recording():
    for i in range(6):
        ranges = [None] * 360
        ranges[180] = 1.0
        yield dict(time=i * .1, encoders=[0, 0] if i < 3 else [i-2, i-2],
                   ranges=ranges, gyro_yaw_rate=.02 if i < 3 else 1.02,
                   stationary=i < 3)


class TestMappingReplay(unittest.TestCase):
    def run_replay(self, records, options=OPTIONS):
        with patch.object(replay.config, 'GRID_WIDTH', 60), \
             patch.object(replay.config, 'GRID_HEIGHT', 60):
            return replay.replay(records, (0, 0, 0), options)

    def test_maps_and_pose_comparison_use_recorded_sensors(self):
        maps, poses, stats = self.run_replay(recording())
        self.assertAlmostEqual(stats['gyro_bias'], .02)
        self.assertAlmostEqual(poses[-1][3], 0.0)
        self.assertAlmostEqual(poses[-1][6], .3)
        self.assertEqual(stats['samples'], 6)
        self.assertGreater(maps[0].count(replay.OCCUPIED), 0)
        self.assertGreater(maps[1].count(replay.OCCUPIED), 0)
        self.assertIn('not ground-truth', stats['note'])

    def test_disabled_fusion_matches_encoder_map(self):
        maps, poses, stats = self.run_replay(recording(), None)
        self.assertEqual(maps[0].grid, maps[1].grid)
        self.assertEqual(stats['max_heading_difference_rad'], 0)
        self.assertFalse(stats['gyro_enabled'])

    def test_bad_time_or_scan_is_rejected(self):
        for key, value in [('time', 0), ('time', math.nan), ('ranges', []),
                           ('stationary', 'yes')]:
            records = list(recording())
            records[1][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.run_replay(records)
        with self.assertRaises(ValueError):
            self.run_replay([])

    def test_missing_encoders_skip_mapping_and_rebase(self):
        records = list(recording())
        records[3]['encoders'] = None
        _, _, stats = self.run_replay(records)
        self.assertEqual(stats['skipped_map_scans'], 2)

    def test_cli_exports_artifacts_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'recording.jsonl'
            source.write_text('\n'.join(json.dumps(r) for r in recording()), encoding='utf-8')
            output = root / 'maps'
            options_file = root / 'gyro.json'
            options_file.write_text(json.dumps(OPTIONS), encoding='utf-8-sig')
            cmd = [sys.executable, str(Path(replay.__file__)), str(source),
                   '--output', str(output), '--start-pose', '0', '0', '0',
                   '--gyro-options-file', str(options_file)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            stats = json.loads((output / 'summary.json').read_text())
            self.assertAlmostEqual(stats['gyro_bias'], .02)
            self.assertTrue((output / 'encoder.pgm').read_text().startswith('P2\n'))
            self.assertEqual(len((output / 'poses.csv').read_text().splitlines()), 7)
            second = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            self.assertNotEqual(second.returncode, 0)
            self.assertIn('already exist', second.stderr)


if __name__ == '__main__':
    unittest.main()
