"""Independent source-time simulation using the shared current-gait plant/PID."""
import math
from oslbench.apps.current_gait import parse_args
from oslbench.sim.current_gait import build, require_backend


class LiveKnee:
    joint_name = 'knee'
    @staticmethod
    def prepare(feedback='actuator-difference', plant='full-leg'):
        """Load the plant before acquisition; no source samples or physics advance."""
        args = parse_args(['--controller', 'knee', '--feedback-mode', feedback])
        mj, _, stub = require_backend('mujoco')
        ctx = build(args, mj, stub, live_reference_deg=0., plant=plant)
        if ctx is None:
            raise ValueError('Model preconditions failed')
        return args, ctx

    def __init__(self, metadata, first, write_step=lambda row: None, feedback='actuator-difference', prepared=None):
        if not first['encoder_valid']:
            raise ValueError('First encoder sample must be valid for initialization')
        self.sign = metadata['flexion_positive_multiplier']
        self.args, self.ctx = prepared if prepared is not None else self.prepare(feedback)
        self.sim, self.src, self.bench = (self.ctx[k] for k in ('sim', 'src', 'bench'))
        joint=self.bench.model.joint(self.joint_name)
        self.joint_qpos=int(joint.qposadr[0])
        self.joint_dof=int(joint.dofadr[0])
        self.dt = self.ctx['dt']
        self.origin = first['time_s']
        self.previous = None
        self.k = 0
        self.write_step = write_step
        self.src.reset()
        self.src.ref[0] = math.radians(self.sign*first['reference_deg'])
        self.sim.reset(math.radians(self.sign*first['actual_deg']), ref_vel0=0.)
        self.last = {}
        self.previous_q = self.q

    @property
    def q(self):
        return math.degrees(float(self.bench.data.qpos[self.joint_qpos]))

    def accept(self, row):
        if self.previous is not None:
            if row['sequence'] != self.previous['sequence']+1:
                raise ValueError('Sample gap: reference history incomplete; start a new run')
            if row['time_s'] <= self.previous['time_s']:
                raise ValueError('Nonmonotonic source time')
            if row['time_s']-self.previous['time_s'] > .1:
                raise ValueError('Source interval exceeds 100 ms; reference history uncertain')
        elapsed = row['time_s']-self.origin
        while self.k*self.dt < elapsed-1e-10:
            self.previous_q = self.q
            target = float(self.src.ref[0])
            self.sim.step(target, self.k)
            s, layer = self.src.last, self.sim.last_layer
            self.k += 1
            self.last = dict(source_time_s=self.origin+self.k*self.dt,
                step_input_time_s=self.origin+(self.k-1)*self.dt,
                physics_step=self.k, control_update=s['control_update'],
                target_deg=math.degrees(target), sim_angle_deg=self.q,
                sim_tracking_error_deg=math.degrees(target)-self.q,
                **{self.joint_name+'_velocity_rad_s':float(self.bench.data.qvel[self.joint_dof])},
                current_requested_A=s['current_request_A'], current_applied_A=s['current_applied_A'],
                current_modelled_A=s['current_actual_modelled_A'], layer_current_A=s['layer_rotor_current_A'],
                current_clipped=s['saturated'], pid_p_A=s['p_A'], pid_i_A=s['i_A'], pid_d_A=s['d_A'],
                actuator_torque_Nm=layer.tau_m, friction_torque_Nm=layer.tau_f,
                belt_actuator_torque_Nm=layer.tau_a, **{self.joint_name+'_drive_torque_Nm':layer.tau_j},
                actuator_angle_rad=layer.theta_a, actuator_velocity_rad_s=layer.theta_a_dot,
                belt_deflection_rad=layer.theta_s, belt_stiffness_Nm_per_rad=layer.K_s,
                belt_energy_J=layer.U, **{self.joint_name+'_servo_force_Nm':self.sim.knee_servo_force()})
            self.last['belt_fit_exceeded'] = int(abs(layer.theta_s)>.055)
            if not all(math.isfinite(v) for v in self.last.values()):
                raise ValueError('Nonfinite simulation state')
            self.write_step(self.last)
        # Compare at the exact measurement time using the two bracketing states.
        # Commands arriving inside a physics interval take effect at the next grid boundary.
        fraction = max(0., min(1., (elapsed-(self.k-1)*self.dt)/self.dt)) if self.k else 1.
        angle = self.previous_q+(self.q-self.previous_q)*fraction if self.k else self.q
        hardware = self.sign*row['actual_deg']
        target = self.sign*row['reference_deg']
        result = dict(source_time_s=row['time_s'], sequence=row['sequence'], target_deg=target,
            hardware_angle_deg=hardware if row['encoder_valid'] else None, sim_angle_deg=angle, encoder_valid=row['encoder_valid'],
            difference_deg=angle-hardware if row['encoder_valid'] else None,
            hardware_tracking_error_deg=target-hardware if row['encoder_valid'] else None,
            sim_tracking_error_deg=target-angle,
            hardware_requested_current_A=row['requested_current_A'],
            hardware_commanded_current_A=row['commanded_current_A'])
        self.src.ref[0] = math.radians(target)
        self.previous = row
        return result
