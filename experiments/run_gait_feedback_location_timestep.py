"""One refined C run; saved baseline is read-only. No coarse/A/B execution path."""
from pathlib import Path
import sys
import json
import hashlib
from dataclasses import asdict
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from experiments import run_gait_feedback_location_estimator as previous
from oslbench.feedback_location_diagnostic import ActuatorDifferenceSource

ab = previous.ab
gait = ab.gait
OUT = ROOT/'build/gait_feedback_location_timestep_convergence'
BASE = previous.OUT/'variant_C'
DT = .00025


def build_refined(mj):
    """Build normally, then change only in-memory timestep and dependent sampling."""
    ab.require_real(mj)
    if mj.__version__ != '3.12.0':
        raise RuntimeError('Requires real MuJoCo 3.12.0')
    args = ab.arguments()
    ctx = gait.build(args, mj, False)
    if ctx is None:
        raise RuntimeError('Production model preconditions failed')
    saved = json.loads((BASE/'run_metadata.json').read_text())
    md = gait.metadata(ctx, args, 'ideal')
    for key in previous.CONFIG_KEYS:
        if md[key] != saved[key]:
            raise RuntimeError('Saved baseline configuration mismatch: '+key)
    old = ab.trace(BASE/'gait_trace.csv')
    assert len(old['time_s']) == ctx['n'] == 2410
    assert np.allclose(np.degrees(ctx['res']['ref_rad']),old['reference_deg'],atol=1e-12,rtol=0)
    # No step of the coarse context is executed. Both integrations read opt.timestep.
    ctx['bench'].model.opt.timestep = DT
    ctx['bench'].timestep = DT
    ctx['dt'] = DT
    ctx['steps_per_update'] = int(round((1/gait.CONTROL_HZ)/DT))
    assert ctx['steps_per_update'] == 4
    assert ctx['steps_per_update']*DT == .001
    ctx['res'] = gait.resample_reference(ctx['ref'],DT,1,'cubic',verbose=False)
    ctx['n'] = ctx['res'].n
    assert ctx['n'] == 4820
    assert np.allclose(np.degrees(ctx['res']['ref_rad'][::2]),old['reference_deg'],atol=1e-12,rtol=0)
    src = ActuatorDifferenceSource(ctx['cfg'],ctx['audit'],ctx['res']['ref_rad'],4,ctx['sim'].layer)
    src.dt = DT
    ctx['src'] = ctx['sim'].current_source = src
    ctx['ident'] = gait.model_identity(ctx['bench'],mj,False)
    return ctx


def main():
    import mujoco
    path = OUT/'variant_C_dt_0p25ms'
    if path.exists():
        raise FileExistsError('Refusing rerun or overwrite of refined C')
    manifest = json.loads((previous.OUT/'run_metadata.json').read_text())
    # Authenticate unchanged simulation implementation and protected physical inputs.
    for name,h in manifest['source_sha256'].items():
        if name.startswith(('oslbench/','experiments/','models/','tests/oracle/')) or name=='build/AB19_knee_gait_reference.csv':
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
    ctx = build_refined(mujoco)
    old = ab.trace(BASE/'gait_trace.csv')
    old_rows = np.array([old[c] for c in gait.CSV_HEADER]).T
    # Recompute all saved numerical legacy metrics from saved rows; no integration.
    coarse = dict(ctx,dt=.0005,n=2410,steps_per_update=2)
    saved_metrics = dict(ab.read_csv(BASE/'gait_metrics.csv')[1])
    checks=[]
    for name,value in gait.metrics(old_rows,coarse,ab.arguments()).items():
        if isinstance(value,(float,int)) and not isinstance(value,bool):
            delta=abs(float(saved_metrics[name])-value)
            checks.append([name,delta,delta<=1e-12])
    ab.write_csv(OUT/'saved_baseline_checks.csv',['metric','absolute_difference','passed'],checks)
    assert all(row[-1] for row in checks)
    # Reserve before integration so failures cannot be silently rerun.
    path.mkdir()
    extras=[]
    physical_times=[]
    def capture(k,q,s,layer):
        extras.append([s[c] for c in ab.EXTRA[:11]]+
            [s['control_update']*.001,s['ref_deg'],s['meas_deg'],s['error_deg']]+
            [s[c] for c in previous.C_EXTRA])
        physical_times.append(float(ctx['bench'].data.time))
    rows=gait.run(ctx,ab.arguments(),on_frame=capture)
    ab.write_csv(path/'gait_trace.csv',list(gait.CSV_HEADER)+list(ab.EXTRA)+list(previous.C_EXTRA),
                 [list(r)+e for r,e in zip(rows,extras)])
    ab.write_csv(path/'observed_physics_time.csv',['post_step_time_s'],[[t] for t in physical_times])
    assert np.isfinite(rows).all() and np.isfinite(extras).all()
    assert abs(physical_times[-1]-1.205)<1e-12
    assert np.allclose(np.diff([0.]+physical_times),DT,atol=1e-12,rtol=0)
    assert ctx['src'].updates==1205
    assert ctx['bench'].limits_unchanged()
    met=gait.metrics(rows,ctx,ab.arguments())
    ab.write_csv(path/'gait_metrics.csv',['metric','value'],met.items())
    md=gait.metadata(ctx,ab.arguments(),'ideal')
    md.update(variant='C',experiment='timestep_convergence',
        derivative_on='reference_backward_difference_minus_actuator_angle_backward_difference',
        derivative_period_s=.001,first_derivative_suppressed=True,
        plant_parameters=asdict(ctx['sim'].layer.p),layer_substeps=ctx['sim'].layer.substeps,
        baseline_path=str(BASE),baseline_rerun=False,refined_C_runs=1,
        observed_horizon_s=physical_times[-1],
        source_sha256=ab.fingerprints(),mujoco=mujoco.__version__,numpy=np.__version__,python=sys.version,
        runtime_override='compiled model opt.timestep and BenchModel cached timestep; no XML change',
        reference_sampling='same 101-point NaturalCubic, evaluated at k*0.00025 on [0,1.205)',
        command_period_s=.001)
    ab.write_json(path/'run_metadata.json',md)
    print('Refined C ran once: 4820 steps, 1205 updates, horizon',physical_times[-1])
    print(json.dumps(met,indent=2))


if __name__=='__main__':
    main()
