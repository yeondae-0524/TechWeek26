"""Replay recorded encoder/LiDAR/gyro JSONL without Webots or robot commands.

Each record: time (seconds), encoders ([left, right] or null), ranges (360
metres, null rays mean no-return), gyro_yaw_rate (rad/s or null), and optional
stationary (bool, explicitly confirmed stopped). No pose ground truth is used.
Outputs compare encoder and optional gyro maps; differences are NOT accuracy.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'rescue_robot'))
import config
from interfaces import FREE, OCCUPIED, UNKNOWN
from localization import GyroHeading, Localizer
from mapping import OccupancyGrid


def replay(records, start_pose, gyro_options=None):
    """Return two maps, per-sample poses, and comparison statistics.

    Scan records must be at the actual acquisition cadence; duplicate scans
    are not deduplicated automatically. Configuration comes from this checkout.
    """
    maps = [OccupancyGrid.centered_on(*start_pose[:2], config.GRID_WIDTH,
                                    config.GRID_HEIGHT, config.GRID_RESOLUTION)
            if config.GRID_ORIGIN is None else
            OccupancyGrid(config.GRID_WIDTH, config.GRID_HEIGHT,
                          config.GRID_RESOLUTION, config.GRID_ORIGIN)
            for _ in range(2)]
    localizers = [Localizer(start_pose, config.WHEEL_RADIUS, config.AXLE_LENGTH,
                           config.encoder_to_rad(), gyro_heading=gyro)
                  for gyro in (None, GyroHeading(**gyro_options) if gyro_options else None)]
    poses = []
    previous_time = None
    max_difference = 0.0
    skipped = 0
    for record in records:
        now = record['time']
        if not isinstance(now, (float, int)) or not math.isfinite(now):
            raise ValueError('time must be finite')
        if previous_time is not None and now <= previous_time:
            raise ValueError('timestamps must increase strictly')
        stationary = record.get('stationary', False)
        if not isinstance(stationary, bool):
            raise ValueError('stationary must be a boolean')
        encoders = record['encoders']
        if encoders is not None and (len(encoders) != 2 or
                                    not all(math.isfinite(v) for v in encoders)):
            raise ValueError('encoders must be two finite numbers or null')
        ranges = record['ranges']
        if len(ranges) != config.LIDAR_EXPECTED_RESOLUTION:
            raise ValueError('scan length does not match configured LiDAR')
        ranges = [math.inf if value is None else float(value) for value in ranges]
        dt = None if previous_time is None else now - previous_time
        # Do not insert scans at an unknown pose during encoder loss/rebase.
        mapping_valid = (encoders is not None and
                         (previous_time is None or all(loc.odometry._prev is not None
                                                       for loc in localizers)))
        pair = []
        for grid, loc in zip(maps, localizers):
            pose = loc.update(encoders, record.get('gyro_yaw_rate'), dt,
                              stationary=stationary)
            pair.extend(pose)
            if mapping_valid:
                grid.insert_scan(pose, ranges, config.LIDAR_EXPECTED_FOV,
                                 config.LIDAR_MAX_RANGE, min_range=config.LIDAR_MIN_RANGE)
        if not mapping_valid:
            skipped += 1
        heading_diff = math.atan2(math.sin(pair[5] - pair[2]), math.cos(pair[5] - pair[2]))
        max_difference = max(max_difference, abs(heading_diff))
        poses.append((now, *pair, heading_diff, localizers[1].heading_source))
        previous_time = now
    if not poses:
        raise ValueError('recording is empty')
    gyro = localizers[1].gyro_heading
    stats = dict(samples=len(poses), skipped_map_scans=skipped,
                 gyro_enabled=gyro is not None, gyro_bias=None if gyro is None else gyro.bias,
                 max_heading_difference_rad=max_difference,
                 note='Encoder/gyro disagreement is not ground-truth position error.',
                 origin=maps[0].origin, resolution=maps[0].resolution,
                 width=maps[0].width, height=maps[0].height,
                 cell_counts=[{name: grid.count(value) for name, value in
                               [('unknown', UNKNOWN), ('free', FREE), ('occupied', OCCUPIED)]}
                              for grid in maps])
    return maps, poses, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--start-pose', required=True, nargs=3, type=float,
                        metavar=('X', 'Y', 'THETA_RAD'))
    gyro_args = parser.add_mutually_exclusive_group()
    gyro_args.add_argument('--gyro-options', type=json.loads,
                           help='JSON object of explicit GyroHeading tuning parameters')
    gyro_args.add_argument('--gyro-options-file', type=Path,
                           help='UTF-8 JSON file (avoids shell quoting of JSON)')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in args.start_pose):
        parser.error('start pose must be finite')
    options = args.gyro_options
    if args.gyro_options_file is not None:
        options = json.loads(args.gyro_options_file.read_text(encoding='utf-8-sig'))
    names = ('encoder.pgm', 'gyro.pgm', 'poses.csv', 'summary.json')
    if any((args.output / name).exists() for name in names):
        parser.error('output files already exist; choose a new output directory')
    with args.recording.open(encoding='utf-8') as stream:
        maps, poses, stats = replay((json.loads(line) for line in stream if line.strip()),
                                   args.start_pose, options)
    args.output.mkdir(parents=True, exist_ok=True)
    for grid, name in zip(maps, names[:2]):
        grid.save_pgm(args.output / name)
    with (args.output / 'poses.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('time', 'encoder_x', 'encoder_y', 'encoder_theta',
                         'gyro_x', 'gyro_y', 'gyro_theta', 'heading_difference', 'heading_source'))
        writer.writerows(poses)
    (args.output / 'summary.json').write_text(
        json.dumps(stats, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(stats, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
