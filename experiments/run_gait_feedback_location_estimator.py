"""Run C once, using saved A/B as read-only controls. No A/B execution path."""
from pathlib import Path
import sys
import json
import hashlib
from datetime import datetime, timezone
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from experiments import run_gait_feedback_location_ab as ab
from oslbench.feedback_location_diagnostic import ActuatorDifferenceSource

OUT = ROOT/'build/gait_feedback_location_estimator_ab'
C_EXTRA = ('actuator_equivalent_sample_deg', 'actuator_difference_velocity_deg_s',
           'controller_period_s')


def saved_baseline_checks():
    """Recheck SAVED A using the established tolerances; no model run required."""
    a = ab.trace(ab.OUT/'variant_A/gait_trace.csv')
    old = ab.trace(ab.OLD/'gait_trace.csv')
    checks = []
    for col in ab.gait.CSV_HEADER:
        atol, rtol = 1e-8, 6e-9
        if col in ('control_update','current_saturated','pid_derivative_used'):
            atol, rtol = 0., 0.
        if col == 'time_s':
            atol, rtol = 1e-12, 0.
        x, y = a[col], old[col]
        ok = x.shape == y.shape and np.allclose(x,y,atol=atol,rtol=rtol)
        checks.append(['trace',col,float(np.max(abs(x-y))),atol,rtol,bool(ok)])
    met_a = dict(ab.read_csv(ab.OUT/'variant_A/gait_metrics.csv')[1])
    met_old = dict(ab.read_csv(ab.OLD/'gait_metrics.csv')[1])
    for key, value in met_old.items():
        if key.startswith('meta_'):
            continue  # Config/model metadata checked below, not reclassified as metrics.
        actual = met_a.get(key)
        try:
            delta = abs(float(actual)-float(value))
            ok = delta <= 2e-7 + 2e-8*abs(float(value))
        except (TypeError,ValueError):
            delta = None
            ok = actual == value
        checks.append(['metric',key,delta,2e-7,2e-8,ok])
    md_a=json.loads((ab.OUT/'variant_A/run_metadata.json').read_text())
    md_old=json.loads((ab.OLD/'run_metadata.json').read_text())
    for key in CONFIG_KEYS:
        checks.append(['metadata',key,None,0,0,md_a[key]==md_old[key]])
    ab.write_csv(OUT/'baseline_reproduction_checks.csv',
                 ['category','quantity','max_abs_difference','atol','rtol','passed'],checks)
    if not all(r[-1] for r in checks):
        raise RuntimeError('Saved A fails baseline check; C not run')
    return a


CONFIG_KEYS = ('kp_A_per_deg','ki_A_per_deg_s','kd_A_per_deg_per_s',
    'measurement_sign','motor_sign','control_hz','physics_dt_s',
    'physics_steps_per_control_update','init_mode','cycles','n_physics_steps',
    'belt_ratio_n_t','k_t_output_Nm_per_A','k_t_output_choice','current_clip_A',
    'current_loop_mode','current_loop_tau_ms','model_model_xml_sha256',
    'model_mujoco_version','model_nq','model_nv','model_ngeom','model_nmesh',
    'model_knee_armature','model_knee_damping','model_knee_frictionloss')


def main():
    import mujoco
    ab.require_real(mujoco)
    if mujoco.__version__ != '3.12.0':
        raise RuntimeError('This fixed experiment requires MuJoCo 3.12.0')
    if (OUT/'variant_C').exists() or (OUT/'run_metadata.json').exists():
        raise FileExistsError('Existing C outputs; refusing rerun or overwrite')
    OUT.mkdir(exist_ok=True)
    # Include every old A/B file, even historical failure records and analysis scripts.
    protected = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in ab.OUT.rglob('*') if p.is_file()}
    before = ab.fingerprints()
    saved_a = saved_baseline_checks()
    saved_b = ab.trace(ab.OUT/'variant_B/gait_trace.csv')
    ctx = ab.gait.build(ab.arguments(), mujoco, False)
    if ctx is None:
        raise RuntimeError('Gait context failed')
    if ctx['dt'] != .0005 or ctx['steps_per_update'] != 2 or ctx['n'] != 2410:
        raise RuntimeError('Fixed sampling contract mismatch')
    md = ab.gait.metadata(ctx,ab.arguments(),'ideal')
    for name in ('A','B'):
        saved=json.loads((ab.OUT/('variant_'+name)/'run_metadata.json').read_text())
        if any(md[k] != saved[k] for k in CONFIG_KEYS):
            raise RuntimeError('Configuration differs from saved '+name)
    ref = np.degrees(ctx['res']['ref_rad'])
    for saved in (saved_a,saved_b):
        if not np.allclose(ref,saved['reference_deg'],atol=1e-12,rtol=0):
            raise RuntimeError('Reference samples differ')
        if not np.allclose(np.arange(ctx['n'])*ctx['dt'],saved['time_s'],atol=1e-12,rtol=0):
            raise RuntimeError('Time grid differs')
    src=ActuatorDifferenceSource(ctx['cfg'],ctx['audit'],ctx['res']['ref_rad'],
                                ctx['steps_per_update'],ctx['sim'].layer)
    src.dt=ctx['dt'];ctx['src']=ctx['sim'].current_source=src
    extras=[]
    def capture(k,q,s,layer):
        extras.append([s[c] for c in ab.EXTRA[:11]]+
            [s['control_update']*.001,s['ref_deg'],s['meas_deg'],s['error_deg']]+
            [s[c] for c in C_EXTRA])
    # Reserve the path before running: even an interrupted run cannot be overwritten.
    path=OUT/'variant_C';path.mkdir()
    rows=ab.gait.run(ctx,ab.arguments(),on_frame=capture)
    ab.write_csv(path/'gait_trace.csv',list(ab.gait.CSV_HEADER)+list(ab.EXTRA)+list(C_EXTRA),
                 [list(r)+e for r,e in zip(rows,extras)])
    if not np.isfinite(np.asarray(rows)).all() or not np.isfinite(extras).all():
        raise RuntimeError('Nonfinite C; saved output retained, stop interpretation')
    met=ab.gait.metrics(rows,ctx,ab.arguments())
    ab.write_csv(path/'gait_metrics.csv',['metric','value'],met.items())
    md.update(experiment='feedback_location_estimator',variant='C',
        derivative_on='reference_backward_difference_minus_actuator_angle_backward_difference',
        actuator_angle_frame='measurement_sign * degrees(theta_a_pre / n_t)',
        theta_a_definition='actuator output shaft after planetary, before belt',
        derivative_period_s=.001,first_derivative_suppressed=True,
        logging='pre-step actuator samples at controller updates; legacy channels unchanged')
    ab.write_json(path/'run_metadata.json',md)
    if before != ab.fingerprints() or any(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h for p,h in protected.items()):
        raise RuntimeError('Protected files changed')
    ab.write_json(OUT/'run_metadata.json',dict(generated_at=datetime.now(timezone.utc).isoformat(),
        source_sha256=before,ab_artifact_sha256=protected,inputs_unchanged=True,
        A_B_rerun=False,C_runs=1,baseline_reproduced=True,configuration=vars(ab.arguments()),
        mujoco=mujoco.__version__,numpy=np.__version__,python=sys.version))
    print('Saved A baseline check passed. C ran once; A/B not rerun. Protected files unchanged.')
    print(json.dumps(met,indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
