"""Fixed-contract A/B diagnostic. Refuses existing results and stub execution.

Run with .venv/Scripts/python.exe -B experiments/run_gait_feedback_location_ab.py.
The original gait runner owns model construction, reference sampling and stepping.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from experiments import run_current_controlled_gait as gait
from oslbench.control.feedback import FeedbackLocationSource

OUT = ROOT / 'build/gait_feedback_location_ab'
OLD = ROOT / 'build/current_controlled_gait/kp0.75_kd0.03_ideal_manual_rotor_cad_aligned'
KD0 = ROOT / 'build/current_controlled_gait/kp0.75_kd0_ideal_manual_rotor_cad_aligned'
EXTRA = ('reference_derivative_deg_s', 'joint_difference_velocity_deg_s',
         'derivative_feedback_velocity_deg_s', 'reference_derivative_current_A',
         'measurement_derivative_current_A', 'derivative_decomposition_residual_A',
         'pre_step_knee_rad', 'pre_step_knee_velocity_rad_s',
         'pre_step_actuator_rad', 'pre_step_actuator_velocity_rad_s',
         'controller_updated', 'controller_sample_time_s',
         'controller_reference_deg', 'controller_measurement_deg', 'controller_error_deg')


def arguments():
    return argparse.Namespace(csv=gait.DEFAULT_REFERENCE_CSV, cycles=1, verbose=False,
        kp=.75, ki=0., kd=.03, kt='manual_rotor', belt='cad', current_clip=60.,
        current_loop='ideal', current_loop_tau_ms=0., derivative_on='error', init='aligned')


def require_real(mj):
    if getattr(mj, '__stub__', False) or not getattr(mj, '__version__', None):
        raise RuntimeError('REFUSING experiment: real MuJoCo is required')


def variant_path(mode, root=OUT):
    if mode not in ('A', 'B'):
        raise ValueError('Unknown variant')
    root = Path(root).resolve()
    if root != OUT.resolve():
        raise ValueError('All outputs must remain under build/gait_feedback_location_ab')
    path = root / ('variant_' + mode)
    if path.exists():
        raise FileExistsError(f'Refusing to overwrite {path}')
    return path


def read_csv(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith('#')]
    if len(rows) < 2 or any(len(r) != len(rows[0]) for r in rows[1:]):
        raise ValueError(f'Malformed CSV: {path}')
    return rows[0], rows[1:]


def trace(path):
    h, r = read_csv(path)
    a = np.asarray(r, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError(f'Nonfinite saved trace: {path}')
    return dict(zip(h, a.T))


def write_json(path, value):
    with Path(path).open('x', encoding='utf-8') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def write_csv(path, header, rows):
    with Path(path).open('x', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fingerprints():
    from oslbench.paths import source_files
    paths = list(source_files(ROOT))
    paths += list((ROOT/'tests').glob('*.py')) + list((ROOT/'tests/oracle').rglob('*'))
    paths += [ROOT/'models/osl_v2_bench.xml', Path(gait.DEFAULT_REFERENCE_CSV)]
    paths += list((ROOT/'build/current_controlled_gait').rglob('*'))
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in paths if p.is_file()}


def run_variant(mode, mj):
    require_real(mj)
    path = variant_path(mode)
    args = arguments()
    ctx = gait.build(args, mj, False)
    if ctx is None:
        raise RuntimeError('Production gait context construction failed')
    src = FeedbackLocationSource(ctx['cfg'], ctx['audit'], ctx['res']['ref_rad'],
                                ctx['steps_per_update'], ctx['sim'].layer, mode)
    src.dt = ctx['dt']
    ctx['src'] = ctx['sim'].current_source = src
    extra = []

    def capture(k, q, s, layer):
        extra.append([s[c] for c in EXTRA[:11]] +
                     [s['control_update'] * ctx['steps_per_update'] * ctx['dt'],
                      s['ref_deg'], s['meas_deg'], s['error_deg']])

    rows = gait.run(ctx, args, on_frame=capture)
    path.mkdir()
    write_csv(path/'gait_trace.csv', list(gait.CSV_HEADER)+list(EXTRA),
              [list(r)+e for r, e in zip(rows, extra)])
    if not np.isfinite(np.asarray(rows)).all():
        write_json(path/'failure.json', {'reason':'Nonfinite trajectory; analysis stopped'})
        raise RuntimeError('Nonfinite trajectory')
    met = gait.metrics(rows, ctx, args)
    write_csv(path/'gait_metrics.csv', ['metric', 'value'], met.items())
    md = gait.metadata(ctx, args, 'ideal')
    md.update(experiment='feedback_location_ab', variant=mode,
              derivative_on='joint_error_backward_difference' if mode == 'A' else
              'reference_backward_difference_minus_instantaneous_actuator_velocity',
              derivative_reference='control-sample backward difference; first update suppressed',
              feedback_frame='measurement_sign * degrees(theta_a_dot / n_t)',
              actuator_state_frame='post-planetary output, pre-belt; not motor rotor',
              generated_at=datetime.now(timezone.utc).isoformat(),
              python=platform.python_version(), numpy=np.__version__,
              logging='legacy channels unchanged; pre_step_* explicitly sampled before layer.advance')
    write_json(path/'run_metadata.json', md)
    return ctx, rows, met


def baseline_values(met, metadata):
    """Historical metrics CSV also embeds metadata with a meta_ prefix."""
    return dict(met, **{'meta_'+key: value for key, value in metadata.items()})


def compare_baseline(ctx, rows, met, filename='baseline_reproduction_checks.csv'):
    old = trace(OLD/'gait_trace.csv')
    new = dict(zip(gait.CSV_HEADER, np.asarray(rows).T))
    checks = []
    # Nine-significant-digit historical CSV. Tolerances are fixed before running.
    for col in gait.CSV_HEADER:
        x, y = new[col], old[col]
        exact = col in ('control_update', 'current_saturated', 'pid_derivative_used')
        atol, rtol = (0., 0.) if exact else (1e-8, 6e-9)
        if col == 'time_s':
            atol, rtol = 1e-12, 0.
        ok = x.shape == y.shape and np.allclose(x, y, atol=atol, rtol=rtol)
        delta = float(np.max(np.abs(x-y))) if x.shape == y.shape else None
        checks.append(['trace', col, delta, atol, rtol, bool(ok)])
    now = gait.metadata(ctx, arguments(), 'ideal')
    values = baseline_values(met, now)
    _, saved = read_csv(OLD/'gait_metrics.csv')
    for key, value in saved:
        if key not in values:
            checks.append(['metric', key, None, 2e-7, 2e-8, False])
            continue
        try:
            a, b = float(values[key]), float(value)
            delta = abs(a-b)
            ok = delta <= 2e-7 + 2e-8*abs(b)
        except (ValueError, TypeError):
            delta = None
            ok = str(values[key]).lower() == value.lower()
        checks.append(['metric', key, delta, 2e-7, 2e-8, ok])
    md = json.loads((OLD/'run_metadata.json').read_text())
    for key in ('model_model_xml_sha256', 'model_mujoco_version', 'physics_dt_s',
                'control_hz', 'init_mode', 'belt_ratio_n_t', 'k_t_output_Nm_per_A'):
        checks.append(['metadata', key, None, 0, 0, now[key] == md[key]])
    write_csv(OUT/filename,
              ['category','quantity','max_abs_difference','atol','rtol','passed'], checks)
    return all(r[-1] for r in checks)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--recheck-saved-A', action='store_true',
                    help='Recheck an existing A without rerunning; B must not exist')
    cli = ap.parse_args()
    import mujoco
    require_real(mujoco)
    # Validate all historical inputs before spending a simulation or writing output.
    trace(OLD/'gait_trace.csv')
    trace(KD0/'gait_trace.csv')
    read_csv(OLD/'gait_metrics.csv')
    json.loads((OLD/'run_metadata.json').read_text())
    if not cli.recheck_saved_A:
        variant_path('A')
    variant_path('B')
    if (OUT/'run_metadata.json').exists():
        raise FileExistsError('Experiment metadata already exists; refusing overwrite')
    before = fingerprints()
    OUT.mkdir(exist_ok=True)
    if cli.recheck_saved_A:
        ctx = gait.build(arguments(), mujoco, False)
        saved_a = trace(OUT/'variant_A/gait_trace.csv')
        rows = np.column_stack([saved_a[c] for c in gait.CSV_HEADER]).tolist()
        met = dict(read_csv(OUT/'variant_A/gait_metrics.csv')[1])
        check_name = 'baseline_reproduction_recheck.csv'
    else:
        ctx, rows, met = run_variant('A', mujoco)
        check_name = 'baseline_reproduction_checks.csv'
    if not compare_baseline(ctx, rows, met, check_name):
        write_json(OUT/'failure.json', {'reason':'A baseline mismatch; B not run'})
        raise RuntimeError('A failed reproduction; B NOT RUN')
    print('Variant A: baseline reproduction PASSED. Starting B.', flush=True)
    run_variant('B', mujoco)
    after = fingerprints()
    if before != after:
        raise RuntimeError('Protected inputs/source fingerprints changed during run')
    write_json(OUT/'run_metadata.json', dict(generated_at=datetime.now(timezone.utc).isoformat(),
        source_sha256=before, inputs_unchanged=True, baseline_reproduced=True,
        production_defaults_changed=False, real_mujoco=mujoco.__version__,
        python=platform.python_version(), numpy=np.__version__,
        output_root=str(OUT), configuration=vars(arguments())))
    print('A and B completed. Original inputs unchanged.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
