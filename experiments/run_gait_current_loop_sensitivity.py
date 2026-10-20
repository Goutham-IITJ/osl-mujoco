"""Fixed Variant-C current-loop sensitivity; no controller/plant equations duplicated."""
from pathlib import Path
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import numpy as np
from oslbench.sim import current_gait as gait

TAU_MS = (0., 1., 2., 5., 10., 20.)
DEFAULT_OUT = ROOT / 'build/gait_current_loop_sensitivity'
BASELINE = ROOT / 'build/gait_feedback_location_estimator_ab/variant_C/gait_trace.csv'


def arguments(tau_ms):
    if tau_ms not in TAU_MS:
        raise ValueError('Use the previously studied illustrative time constants')
    return argparse.Namespace(csv=gait.DEFAULT_REFERENCE_CSV, cycles=1, verbose=False,
        kp=.75, ki=0., kd=.03, kt='manual_rotor', belt='cad', current_clip=60.,
        current_loop='ideal' if tau_ms == 0 else 'first_order',
        current_loop_tau_ms=tau_ms, derivative_on='error', init='aligned',
        feedback_mode='actuator-difference')


def save_json(path, data):
    with path.open('x', encoding='utf-8') as f:
        json.dump(data, f, indent=2, allow_nan=False)


def save_csv(path, fields, rows):
    with path.open('x', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        writer.writerows(rows)


def fingerprints():
    # All existing evidence, including images, plus source/configuration inputs.
    paths = [p for folder in ('build', 'models', 'tests', 'oslbench', 'experiments', 'docs')
             for p in (ROOT / folder).rglob('*') if p.is_file()
             and '__pycache__' not in p.parts and p.suffix != '.pyc']
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}


def summarize(rows, ctx, tau):
    a = np.asarray(rows, dtype=float)
    f = lambda name: a[:, gait.CSV_HEADER.index(name)]
    peak = lambda name: float(np.max(np.abs(f(name))))
    error = f('reference_deg') - f('knee_angle_deg')
    delta = f('current_request_A') - f('current_actual_modelled_A')
    outside = np.abs(f('theta_s_rad')) > .055
    lo, hi = np.degrees(ctx['bench'].knee_ctrlrange)
    return dict(tau_ms=tau, loop=arguments(tau).current_loop, rows=len(rows),
        rms_error_deg=float(np.sqrt(np.mean(error**2))), peak_error_deg=float(max(abs(error))),
        peak_requested_A=peak('current_request_A'), peak_applied_A=peak('current_applied_A'),
        peak_modelled_A=peak('current_actual_modelled_A'),
        rms_request_minus_modelled_A=float(np.sqrt(np.mean(delta**2))),
        max_request_minus_modelled_A=float(max(abs(delta))),
        clipped_samples=int(np.count_nonzero(f('current_saturated'))),
        clipped_duration_s=float(np.count_nonzero(f('current_saturated'))*ctx['dt']),
        peak_knee_belt_Nm=peak('belt_torque_joint_side_Nm'),
        max_deflection_rad=peak('theta_s_rad'), outside_fit_samples=int(outside.sum()),
        outside_fit_duration_s=float(outside.sum()*ctx['dt']),
        nonfinite_cells=int(np.count_nonzero(~np.isfinite(a))),
        rom_violation_samples=int(np.count_nonzero((f('knee_angle_deg') < lo-1e-6) |
                                                  (f('knee_angle_deg') > hi+1e-6))),
        servo_nonzero_samples=int(np.count_nonzero(f('knee_servo_force_Nm'))),
        controller_updates=ctx['src'].updates, current_loop_steps=ctx['src'].loop.steps)


def plots(out, cases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    note = 'SIMULATION ONLY: illustrative time constants, not measured hardware values.'
    for filename, channels in (
        ('01_tracking.png', ('knee_angle_deg', 'tracking_error_deg')),
        ('03_loading.png', ('belt_torque_joint_side_Nm', 'theta_s_rad'))):
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        for tau, a in cases:
            t = a[:, 0]
            for ax, channel in zip(axes, channels):
                ax.plot(t, a[:, gait.CSV_HEADER.index(channel)], label=f'{tau:g} ms', lw=1)
                ax.set_ylabel(channel)
                ax.grid(alpha=.25)
        if channels[0] == 'knee_angle_deg':
            axes[0].plot(cases[0][1][:, 0], cases[0][1][:, 3], 'k--', label='Reference')
        else:
            for v in (-.055, .055):
                axes[1].axhline(v, color='gray', ls=':', label='Fit range' if v>0 else None)
        for ax in axes:
            ax.legend(ncol=4, fontsize=8)
        axes[-1].set_xlabel('Legacy logged time [s]')
        fig.suptitle('Variant C current-loop sensitivity\n'+note)
        fig.tight_layout()
        fig.savefig(out/filename, dpi=150)
        plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    for ax, (tau, a) in zip(axes.flat, cases):
        for name, label, style in [('current_request_A','Requested','-'),
                                   ('current_applied_A','Applied (clipped command)',':'),
                                   ('current_actual_modelled_A','Modelled actual','--')]:
            ax.plot(a[:,0], a[:,gait.CSV_HEADER.index(name)], style, label=label, lw=1)
        ax.set_title(f'tau = {tau:g} ms')
        ax.set_ylabel('Current [A]')
        ax.set_xlabel('Logged time [s]')
        ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8)
    fig.suptitle(note+'\nEach panel shows its own closed-loop command and response.')
    fig.tight_layout()
    fig.savefig(out/'02_currents.png', dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--outdir', type=Path, default=DEFAULT_OUT)
    args_cli = ap.parse_args()
    out = args_cli.outdir.resolve()
    if not out.is_relative_to(ROOT/'build'):
        raise ValueError('Output must be a new directory under build/')
    if out.exists():
        raise FileExistsError(out)
    import mujoco
    if getattr(mujoco, '__stub__', False) or not getattr(mujoco, '__version__', None):
        raise RuntimeError('Real MuJoCo required')
    before = fingerprints()
    out.mkdir(parents=True)
    save_json(out/'preservation_before.json', before)
    save_json(out/'execution.json', dict(status='RUNNING', taus_ms=TAU_MS,
        mujoco=mujoco.__version__, python=sys.version, baseline=str(BASELINE.relative_to(ROOT))))
    results, cases, fixed = [], [], None
    for tau in TAU_MS:
        case = out/('ideal' if tau == 0 else f'tau_{tau:g}ms')
        case.mkdir()
        args = arguments(tau)
        ctx = gait.build(args, mujoco, False)
        if ctx is None:
            raise RuntimeError('Model checks failed')
        assert ctx['dt'] == .0005 and ctx['steps_per_update'] == 2 and ctx['n'] == 2410
        md = gait.metadata(ctx, args, args.current_loop)
        md.update(plant_parameters=asdict(ctx['sim'].layer.p),
            layer_substeps=ctx['sim'].layer.substeps, current_initial_A=ctx['src'].loop.i0,
            command_period_s=.001, time_constant_provenance='Earlier ankle sensitivity values; NOT hardware measurements',
            source_manifest='../preservation_before.json')
        common = {k:v for k,v in md.items() if k not in
                  ('current_loop_mode','current_loop_tau_ms','current_loop_note')}
        if fixed is None:
            fixed = common
        assert common == fixed, 'A non-current-loop parameter changed'
        save_json(case/'configuration.json', md)
        rows = gait.run(ctx, args)
        a = np.asarray(rows, float)
        save_csv(case/'gait_trace_full_precision.csv', gait.CSV_HEADER, rows)
        summary = summarize(rows, ctx, tau)
        save_json(case/'summary.json', summary)
        if summary['nonfinite_cells']:
            raise RuntimeError('Non-finite trajectory; saved for diagnosis')
        assert ctx['src'].updates == 1205 and ctx['src'].loop.steps == 2410
        assert ctx['bench'].limits_unchanged()
        # Independent recurrence check, not a second plant or controller.
        target = a[:,gait.CSV_HEADER.index('current_applied_A')]
        actual = a[:,gait.CSV_HEADER.index('current_actual_modelled_A')]
        previous = np.r_[0., actual[:-1]]
        expected = target if tau == 0 else target+(previous-target)*math.exp(-.0005/(tau*.001))
        assert np.allclose(actual, expected, atol=1e-12, rtol=1e-12)
        assert np.array_equal(target[::2], target[1::2])
        assert np.allclose(a[:,gait.CSV_HEADER.index('estimated_actuator_output_torque_Nm')],
                           actual*.855, atol=1e-12, rtol=1e-12)
        if tau == 0:
            with BASELINE.open() as f:
                old = list(csv.DictReader(f))
            baseline = np.array([[float(row[c]) for c in gait.CSV_HEADER] for row in old])
            assert np.array_equal(a, baseline), 'Ideal C no longer exactly reproduces saved C'
            save_json(out/'baseline_parity.json', dict(exact=True, rows=2410, columns=29,
                       max_absolute_difference=float(np.max(abs(a-baseline)))))
        results.append(summary)
        cases.append((tau,a))
        print(json.dumps(summary), flush=True)
    save_csv(out/'comparison.csv', list(results[0]), [list(r.values()) for r in results])
    plots(out,cases)
    changed=[name for name,h in before.items()
             if not (ROOT/name).is_file() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=h]
    save_json(out/'preservation_after.json',dict(files_checked=len(before), changed=changed))
    assert not changed, changed
    save_json(out/'completed.json',dict(status='COMPLETE', runs=6, checks='exact baseline parity; fixed configuration; current recurrence; held command; modelled-current torque; counts; preservation'))
    print('Completed:',out)


if __name__ == '__main__':
    main()
