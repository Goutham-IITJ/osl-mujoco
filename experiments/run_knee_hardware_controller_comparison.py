"""Isolated supplied-knee PID study. Never changes production or old evidence."""
from pathlib import Path
import argparse
import ast
import csv
from dataclasses import asdict
import hashlib
import itertools
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from oslbench.sim import current_gait as gait
from oslbench.control.current import CurrentPID
from oslbench.control.gait import GaitCurrentSource
from oslbench.control.feedback import referred_angle, referred_velocity
from oslbench.sim.contracts import GaitContractAdapter


def supplied_pid(path):
    """Extract ONLY the inspected arithmetic class; never import hardware code."""
    tree = ast.parse(path.read_text(encoding='utf-8-sig'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PID')
    scope = {}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    values = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
            try:
                values[n.targets[0].id] = ast.literal_eval(n.value)
            except (ValueError, TypeError):
                pass
    return scope['PID'], values


class StudyPID:
    """Keep P/I knee-side. Only B/C replace D's measured-velocity input."""
    def __init__(self, cfg, mode, hardware_class=None, integral_limit=50.):
        self.cfg, self.mode = cfg, mode
        g = cfg.gains
        self.base = (hardware_class(g.kp_A_per_deg, g.ki_A_per_deg_s,
                                   g.kd_A_per_deg_per_s, integral_limit)
                     if hardware_class else CurrentPID(g, cfg.limits))
        self.hardware = hardware_class is not None
        self.reset()

    def reset(self):
        self.base.reset()
        self.prev_ref = self.prev_angle = None
        self.angle = self.velocity = 0.
        self.max_integral = 0.
        self.last = dict(error_deg=0., p_A=0., i_A=0., d_A=0., raw_A=0.,
                         raw_signed_A=0., cmd_A=0., saturated=False, derivative_used=False)

    def update(self, ref, meas, dt):
        used = self.prev_ref is not None
        if self.hardware:
            raw, p, i, d, err = self.base.update(ref, meas, dt)
        else:
            self.base.update(ref, meas, dt)
            s = self.base.last
            raw, p, i, d, err = [s[k] for k in ('raw_A', 'p_A', 'i_A', 'd_A', 'error_deg')]
        if self.mode != 'A':
            vr = (ref-self.prev_ref)/dt if used else 0.
            vf = self.velocity if self.mode == 'B' else ((self.angle-self.prev_angle)/dt if used else 0.)
            d = self.cfg.gains.kd_A_per_deg_per_s*(vr-vf) if used else 0.
            raw = p+i+d
        signed = self.cfg.gains.encoder_to_motor_sign*raw
        clip = self.cfg.limits.sim_clip_A
        cmd = max(-clip, min(clip, signed))
        self.last = dict(error_deg=err, p_A=p, i_A=i, d_A=d, raw_A=raw,
                         raw_signed_A=signed, cmd_A=cmd, saturated=cmd != signed,
                         derivative_used=used)
        self.max_integral = max(self.max_integral, abs(self.base.integral_deg_s))
        self.prev_ref, self.prev_angle = ref, self.angle
        return cmd


class StudySource(GaitCurrentSource):
    def __call__(self, k, q, qdot):
        self.pid.angle = referred_angle(self.layer.theta_a, self.layer.p.n_t, self.cfg.measurement_sign)
        self.pid.velocity = referred_velocity(self.layer.theta_a_dot, self.layer.p.n_t, self.cfg.measurement_sign)
        return super().__call__(k, q, qdot)


def arguments(gains, kt):
    return argparse.Namespace(csv=gait.DEFAULT_REFERENCE_CSV, cycles=1, verbose=False,
        kp=gains[0], ki=gains[1], kd=gains[2], kt=kt, belt='cad', current_clip=60.,
        current_loop='ideal', current_loop_tau_ms=0., derivative_on='error',
        init='aligned', feedback_mode='legacy')


def setup(gains, kt, hz, mode, mj, hardware_class=None, integral_limit=50.):
    args = arguments(gains, kt)
    ctx = gait.build(args, mj, False)
    if ctx is None:
        raise RuntimeError('Model build failed')
    steps = round(1/hz/ctx['dt'])
    assert abs(steps*ctx['dt']-1/hz) < 1e-12
    src = StudySource(ctx['cfg'], ctx['audit'], ctx['res']['ref_rad'], steps)
    src.layer = ctx['sim'].layer
    src.pid = StudyPID(ctx['cfg'], mode, hardware_class, integral_limit)
    src.dt = ctx['dt']
    ctx['src'], ctx['steps_per_update'] = src, steps
    ctx['sim'].current_source = GaitContractAdapter(src, src.layer)
    return ctx, args


def write_json(path, obj):
    with path.open('x', encoding='utf-8') as f:
        json.dump(obj, f, indent=2, allow_nan=False)


def write_csv(path, fields, rows):
    with path.open('x', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(fields)
        w.writerows(rows)


def inventory(excluded):
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('oslbench', 'experiments', 'tests', 'docs', 'models', 'build')
            for p in (ROOT/folder).rglob('*') if p.is_file() and not p.is_relative_to(excluded)
            and '__pycache__' not in p.parts and p.suffix != '.pyc'}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hardware-script', type=Path, required=True)
    ap.add_argument('--outdir', type=Path, default=ROOT/'build/knee_hardware_controller_comparison')
    cli = ap.parse_args()
    out = cli.outdir.resolve()
    assert out.is_relative_to(ROOT/'build')
    out.mkdir()  # Existing output is never overwritten.
    hardware_class, hw = supplied_pid(cli.hardware_script)
    (out/'supplied_knee_controller.txt').write_bytes(cli.hardware_script.read_bytes())
    write_json(out/'supplied_settings.json', hw)
    before = inventory(out)
    write_json(out/'preservation_before.json', before)
    import mujoco
    assert not getattr(mujoco, '__stub__', False)
    assert mujoco.__version__ == '3.12.0'
    summaries, traces = [], {}
    for family, kt, hz, mode in itertools.product(('old', 'knee'), ('manual_rotor', 'script'), (1000, 200), ('A','B','C')):
        gains = (.75,0.,.03) if family == 'old' else (hw['KP'], hw['KI'], hw['KD'])
        name = f'{family}_{kt}_{hz}Hz_{mode}'
        case = out/name
        case.mkdir()
        ctx, args = setup(gains, kt, hz, mode, mujoco,
                          hardware_class if family == 'knee' else None, hw['INTEGRAL_LIMIT_DEG_S'])
        assert ctx['dt'] == .0005 and ctx['n'] == 2410
        config = dict(gains=gains, gain_source='repository ankle-derived' if family=='old' else 'supplied knee script',
            hz=hz, physics_dt_s=ctx['dt'], mode=mode, kt=asdict(ctx['audit']),
            integral_limit_deg_s=None if family=='old' else hw['INTEGRAL_LIMIT_DEG_S'],
            plant=asdict(ctx['sim'].layer.p), substeps=ctx['sim'].layer.substeps,
            model=ctx['ident'], args=vars(args), mujoco=mujoco.__version__,
            note='Model-frame signs +1/+1; ideal current; no hardware sensor faults or aborts emulated.')
        write_json(case/'configuration.json', config)
        applied = []
        controller_errors = []
        def capture(k, q, s, layer):
            applied.append(float(ctx['bench'].data.qfrc_applied[ctx['bench'].knee_dof]))
            if k % ctx['steps_per_update'] == 0:
                controller_errors.append(s['error_deg'])
        rows = gait.run(ctx, args, capture)
        a = np.asarray(rows, float)
        assert np.isfinite(a).all(), name+' nonfinite'
        f = lambda c: a[:, gait.CSV_HEADER.index(c)]
        write_csv(case/'trace.csv', list(gait.CSV_HEADER)+['knee_applied_drive_torque_Nm'],
                  [list(r)+[t] for r,t in zip(rows,applied)])
        met = gait.metrics(rows, ctx, args)
        write_json(case/'metrics.json', met)
        # Verify physical actuation and command scheduling for every case.
        assert np.allclose(f('estimated_actuator_output_torque_Nm'),
                           f('current_actual_modelled_A')*ctx['audit'].k_t_output_Nm_per_A, atol=1e-10)
        assert np.count_nonzero(f('knee_servo_force_Nm')) == 0
        assert np.allclose(applied, f('belt_torque_joint_side_Nm'), atol=1e-10)
        assert ctx['src'].updates == 2410//ctx['steps_per_update']
        for k in range(len(a)):
            assert f('current_applied_A')[k] == f('current_applied_A')[k-k%ctx['steps_per_update']]
        parity = None
        if family == 'old' and kt == 'manual_rotor' and hz == 1000:
            old = ROOT/('build/gait_feedback_location_estimator_ab/variant_C/gait_trace.csv'
                       if mode=='C' else f'build/gait_feedback_location_ab/variant_{mode}/gait_trace.csv')
            with old.open() as fcsv:
                baseline = np.array([[float(r[c]) for c in gait.CSV_HEADER] for r in csv.DictReader(fcsv)])
            parity = float(np.max(np.abs(a-baseline)))
            assert np.allclose(a, baseline, atol=1e-8, rtol=6e-9), (name, parity)
        m = dict(case=name, gains=family, kt_output=ctx['audit'].k_t_output_Nm_per_A,
            hz=hz, feedback=mode, rms_deg=met['rms_error_deg'], peak_error_deg=met['peak_abs_error_deg'],
            lag_ms=met['best_fit_lag_ms'], peak_belt_actuator_Nm=met['peak_belt_torque_ACTUATOR_side_Nm'],
            peak_belt_joint_Nm=met['peak_belt_torque_JOINT_side_Nm'],
            peak_knee_drive_Nm=float(max(abs(np.asarray(applied)))),
            peak_current_A=met['peak_current_actual_modelled_A'], peak_requested_A=met['peak_current_request_A'],
            max_deflection_rad=met['max_abs_theta_s_rad'], outside_fit_steps=int(sum(abs(f('theta_s_rad'))>.055)),
            clipped_steps=int(sum(f('current_saturated'))), rom_violations=met['rom_violation_steps'],
            max_integral_deg_s=ctx['src'].pid.max_integral,
            controller_samples_over_20deg=int(sum(abs(np.asarray(controller_errors))>hw['ABORT_ERROR_DEG'])),
            baseline_max_difference=parity)
        summaries.append(m)
        traces[name] = a
        write_json(case/'summary.json', m)
        print(json.dumps(m), flush=True)
    write_csv(out/'comparison.csv', list(summaries[0]), [list(m.values()) for m in summaries])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for kt in ('manual_rotor','script'):
        fig, axes = plt.subplots(3,2,figsize=(12,9),sharex=True)
        for j,hz in enumerate((1000,200)):
            for family,mode in itertools.product(('old','knee'),('A','C')):
                a=traces[f'{family}_{kt}_{hz}Hz_{mode}']
                for i,c in enumerate(('tracking_error_deg','belt_torque_joint_side_Nm','current_actual_modelled_A')):
                    axes[i,j].plot(a[:,0],a[:,gait.CSV_HEADER.index(c)],label=f'{family} {mode}',lw=.8)
                    axes[i,j].set_ylabel(c)
                    axes[i,j].grid(alpha=.2)
            axes[0,j].set_title(f'{hz} Hz')
            axes[0,j].legend(fontsize=8)
            axes[2,j].set_xlabel('Legacy trace time [s]')
        fig.suptitle(f'Supplied knee PID comparison: {kt}; simulation only')
        fig.tight_layout()
        fig.savefig(out/f'comparison_{kt}.png',dpi=140)
        plt.close(fig)
    after = inventory(out)
    changed = [p for p,h in before.items() if after.get(p) != h]
    write_json(out/'preservation_after.json',dict(files_checked=len(before),changed=changed))
    assert not changed, changed
    write_json(out/'completed.json',dict(runs=len(summaries),status='COMPLETE',python=sys.version))


if __name__ == '__main__':
    main()
