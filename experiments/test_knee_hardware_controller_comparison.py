"""Focused arithmetic/scheduling checks, run directly without changing test registry."""
from pathlib import Path
import sys
from dataclasses import replace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.run_knee_hardware_controller_comparison import StudyPID, supplied_pid, setup
from oslbench.control.gait import KNEE_GAIT_CONFIG
from oslbench.control.current import CurrentPID
from oslbench.control.feedback import FeedbackLocationPID, ActuatorDifferencePID


def main(path):
    hardware, values = supplied_pid(Path(path))
    cfg = replace(KNEE_GAIT_CONFIG, gains=replace(KNEE_GAIT_CONFIG.gains,
                  kp_A_per_deg=values['KP'], ki_A_per_deg_s=values['KI'], kd_A_per_deg_per_s=values['KD']))
    # Compare supplied arithmetic in hardware negative-flexion coordinates with
    # the experiment in positive-flexion coordinates, including saturation/reset.
    study = StudyPID(cfg, 'A', hardware)
    original = hardware(values['KP'],values['KI'],values['KD'],values['INTEGRAL_LIMIT_DEG_S'])
    for k in range(300):
        r, y, dt = (1000. if k<150 else -1000.), 3., .005
        raw,p,i,d,e = original.update(-r,-y,dt)
        expected = np.clip(-raw,-60.,60.)
        assert study.update(r,y,dt) == expected
        assert study.last['i_A'] == -i
    assert study.max_integral == 50.
    study.reset()
    assert study.base.integral_deg_s == 0 and study.update(10.,0.,.005) == 15.0075
    assert not study.last['derivative_used']
    print('PASS supplied PID, sign transform, integral clamp, clipping, first D and reset')
    for mode in 'ABC':
        test = StudyPID(KNEE_GAIT_CONFIG,mode)
        ref = (CurrentPID(KNEE_GAIT_CONFIG.gains,KNEE_GAIT_CONFIG.limits) if mode=='A' else
               FeedbackLocationPID(KNEE_GAIT_CONFIG.gains,KNEE_GAIT_CONFIG.limits,'B') if mode=='B' else
               ActuatorDifferencePID(KNEE_GAIT_CONFIG.gains,KNEE_GAIT_CONFIG.limits))
        for k in range(100):
            test.angle, test.velocity = .1*k*k, 200.*np.sin(k)
            ref.actuator_angle_deg,ref.actuator_velocity_deg_s = test.angle,test.velocity
            x,y = 4.*np.sin(k/12),3.*np.cos(k/10)
            a,b = test.update(x,y,.001),ref.update(x,y,.001)
            assert a == b, (mode,k,a,b)
            for c in ('p_A','i_A','d_A','raw_A','saturated','derivative_used'):
                assert test.last[c] == ref.last[c], (mode,c)
        print('PASS old '+mode+' arithmetic parity')
    # B/C must retain exactly the supplied joint-side P and I with nonzero Ki.
    cases = [StudyPID(cfg,m,hardware) for m in 'ABC']
    for k in range(30):
        for p in cases:
            p.angle,p.velocity = float(k*k), 20.
            p.update(30.,2.*k,.005)
        assert len({p.last['p_A'] for p in cases}) == 1
        assert len({p.last['i_A'] for p in cases}) == 1
    print('PASS nonzero knee integral retained in B/C')
    import mujoco
    from oslbench.sim import current_gait as gait
    for hz in (1000,200):
        ctx,args = setup((1.5,.15,.06),'script',hz,'C',mujoco,hardware)
        ctx['n'] = 30
        a=np.asarray(gait.run(ctx,args))
        cmd=a[:,gait.CSV_HEADER.index('current_applied_A')]
        stride=ctx['steps_per_update']
        assert ctx['src'].updates==30//stride
        assert all(cmd[k] == cmd[k-k%stride] for k in range(30))
        assert np.allclose(a[:,gait.CSV_HEADER.index('estimated_actuator_output_torque_Nm')],cmd*.5701)
    print('PASS 1000/200 Hz schedule and single torque conversion on real MuJoCo')
    print('6 focused checks passed')


if __name__=='__main__':
    main(sys.argv[1])
