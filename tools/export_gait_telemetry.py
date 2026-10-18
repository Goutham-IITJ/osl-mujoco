"""Offline conversion of saved current-gait traces; never imports a simulator."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from oslbench.telemetry_schema import CHANNELS
from oslbench.telemetry.replay import HEADER, ReplaySource, DatasetError

# Channel -> original CSV column, pre/post step. Original files remain untouched.
MAPPING = {
    "knee_angle": ("knee_angle_deg", True),
    "knee_velocity": ("knee_velocity_rad_s", True),
    "actuator_output_angle": ("theta_a_rad", True),
    "actuator_output_velocity": ("theta_a_dot_rad_s", True),
    "electromagnetic_output_torque": ("estimated_actuator_output_torque_Nm", False),
    "belt_joint_torque": ("belt_torque_joint_side_Nm", False),
    "belt_deflection": ("theta_s_rad", False),
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export_gait(trace, metadata, output):
    """Explicit export into a NEW directory under this repository's build/."""
    trace, metadata, output = Path(trace), Path(metadata), Path(output).resolve()
    if not output.is_relative_to((ROOT / 'build').resolve()) or output == (ROOT/'build').resolve():
        raise ValueError("Output must be a new subdirectory under build/")
    if output.exists():
        raise FileExistsError(output)
    m = json.loads(metadata.read_text(encoding='utf-8'))
    if m.get('experiment') != 'run_current_controlled_gait' or m.get('model_backend') != 'real MuJoCo':
        raise DatasetError('Expected recorded real-MuJoCo current-gait metadata')
    dt = m['physics_dt_s']
    if type(dt) not in (float, int) or not math.isfinite(dt) or dt <= 0:
        raise DatasetError('Invalid recorded physics_dt_s')
    with trace.open(newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f, strict=True)
        needed = {'time_s'} | {col for col, _ in MAPPING.values()}
        if reader.fieldnames is None or not needed <= set(reader.fieldnames):
            raise DatasetError('Missing gait trace columns')
        rows = list(reader)
    if len(rows) != m['n_physics_steps']:
        raise DatasetError('Trace count differs from recorded run')
    events = []
    for k, row in enumerate(rows):
        t = float(row['time_s'])
        if not math.isfinite(t) or not math.isclose(t, k*dt, rel_tol=0., abs_tol=1e-12):
            raise DatasetError(f'Gait row {k}: unexpected time grid')
        for name, (column, post) in MAPPING.items():
            value = float(row[column])
            if not math.isfinite(value):
                raise DatasetError(f'Gait row {k}: non-finite {column}')
            if name == 'knee_angle':
                value = math.radians(value)
            events.append((t+dt if post else t, k, name, value))
    events.sort(key=lambda e: (e[0], e[1], e[2]))
    output.mkdir(parents=True, exist_ok=False)
    data = output/'samples.csv'
    with data.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f); writer.writerow(HEADER)
        for seq, (t, k, name, value) in enumerate(events):
            writer.writerow((seq, k, name, repr(t), repr(value), 1))
    channels = {}
    for name, (quantity, unit, frame) in CHANNELS.items():
        channels[name] = dict(quantity=quantity, unit=unit, frame=frame,
            stage='post_step' if MAPPING[name][1] else 'step_input',
            source='simulation', timestamp='time_s',
            sign='Existing MuJoCo knee/actuator positive direction; belt deflection = knee - actuator_output/n_t')
    manifest = dict(schema_version=1, dataset_id=output.name, source='simulation',
        clock=dict(id='source_run_simulation_elapsed', unit='s'),
        sequence='contiguous_zero_based', channels=channels, data_file='samples.csv',
        record_count=len(events), data_sha256=sha(data),
        identity=dict(trace_path=str(trace.resolve()), trace_sha256=sha(trace),
            metadata_path=str(metadata.resolve()), metadata_sha256=sha(metadata),
            source_run_metadata=m, conversion='Knee degrees to radians; k*dt forces and (k+1)*dt kinematics; no interpolation',
            physics_dt_s=dt))
    manifest_path = output/'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    ReplaySource(manifest_path)
    return manifest_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', required=True)
    parser.add_argument('--metadata', required=True)
    parser.add_argument('--outdir', required=True)
    args = parser.parse_args()
    print(export_gait(args.trace, args.metadata, args.outdir))
