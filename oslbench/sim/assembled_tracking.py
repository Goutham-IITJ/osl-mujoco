"""Two independent current PIDs/drivetrains acting on one coupled MuJoCo leg."""
from collections import deque
from pathlib import Path
import hashlib,math
import numpy as np
import mujoco
from oslbench.control.current import CurrentLimits,PIDGains,ConversionAudit,CAD_BELT_RATIO,ANKLE_PAPER_CAD_BELT
from oslbench.control.gait import JointCurrentConfig,make_gait_source
from oslbench.control.compliance import DrivetrainLayer

JOINTS=('knee','ankle')

class AssembledTracking:
    def __init__(self,first,feedback='legacy',ankle_range='cad',on_step=lambda r:None,physics_dt=.0005):
        path=Path(__file__).resolve().parents[2]/'models/osl_v2_bench.xml'
        self.model=mujoco.MjModel.from_xml_path(str(path));self.data=mujoco.MjData(self.model)
        if float(self.model.opt.timestep)!=.0005 or self.model.nq!=2:raise ValueError('Unexpected full-leg model')
        if physics_dt<=0 or abs(.005/physics_dt-round(.005/physics_dt))>1e-8:raise ValueError('Timestep must divide 5ms')
        self.dt=physics_dt;self.model.opt.timestep=physics_dt
        self.qpos={j:int(self.model.joint(j).qposadr[0]) for j in JOINTS}
        self.dof={j:int(self.model.joint(j).dofadr[0]) for j in JOINTS}
        self.original_range=self.model.joint('ankle').range.copy().tolist()
        if ankle_range=='hardware':self.model.joint('ankle').range[:]=np.radians([-31,31])
        elif ankle_range!='cad':raise ValueError('Unknown ankle range')
        # Disable BOTH authored position servos. Only explicit belt torque drives them.
        self.model.actuator_gainprm[:]=0.;self.model.actuator_biasprm[:]=0.
        self.origin=first['reference_time_s'];self.k=0;self.sources={};self.layers={};self.on_step=on_step
        self.range_exceeded=0;self.fit_exceeded=0
        for j,gains in [('knee',(1.5,.15,.06)),('ankle',(2.,0.,.075))]:
            self.data.qpos[self.qpos[j]]=math.radians(first['joints'][j]['actual_deg'])
            limits=CurrentLimits(sim_clip_A=60)
            audit=ConversionAudit(.5701,'script','Provisional actuator-output Nm/A',CAD_BELT_RATIO,'CAD50/11',ANKLE_PAPER_CAD_BELT,limits)
            cfg=JointCurrentConfig(j,PIDGains(*gains),limits,1,'positive CAD coordinate; exported CSV angle mapped directly',
                'Confirmed hardware gains','positive simulated torque; physical mapping provisional',integral_limit_deg_s=50)
            layer=DrivetrainLayer(audit.params,enabled=True,substeps=1)
            layer.seed_rigid(float(self.data.qpos[self.qpos[j]]),0.)
            source=make_gait_source(cfg,audit,np.array([math.radians(first['joints'][j]['reference_deg'])]),round(.005/self.dt),layer,feedback)
            source.dt=self.dt;self.sources[j]=source;self.layers[j]=layer
        mujoco.mj_forward(self.model,self.data)
        self.history=deque([(self.origin,self.angles())],maxlen=200)
        self.last={}
        self.metadata=dict(model=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),feedback=feedback,
            ankle_range_choice=ankle_range,original_ankle_range_rad=self.original_range,
            runtime_ankle_range_rad=self.model.joint('ankle').range.tolist(),physics_dt_s=self.dt,authored_physics_dt_s=.0005,control_hz=200,
            initialization='First measured joint angles assigned at first reference time; zero velocities and belt deflections',
            assumptions='Root knee_prox fixed; no ground/load; physical zero/direction and mount orientation unverified; KT/drivetrain provisional',
            range_note='hardware range is an explicitly labelled experiment override, not verified CAD travel')

    def angles(self):return {j:math.degrees(float(self.data.qpos[self.qpos[j]])) for j in JOINTS}

    def advance(self,t):
        while self.origin+self.k*self.dt<t-1e-10:
            rows=[]
            for j in JOINTS:
                q=float(self.data.qpos[self.qpos[j]]);qd=float(self.data.qvel[self.dof[j]])
                src=self.sources[j];layer=self.layers[j]
                current=src(self.k,q,qd);rec=layer.advance(q,current,self.dt)
                self.data.qfrc_applied[self.dof[j]]=rec.tau_j_mean
                fit=abs(rec.theta_s)>.055;self.fit_exceeded+=int(fit)
                if abs(rec.theta_s)>.15:raise ValueError('Belt deflection >0.15 rad: divergent/out-of-fit simulation stopped')
                rows.append(dict(joint=j,source_time_s=self.origin+(self.k+1)*self.dt,
                    target_deg=math.degrees(src.ref[0]),current_requested_A=src.last['current_request_A'],
                    current_applied_A=src.last['current_applied_A'],current_clipped=src.last['saturated'],
                    pid_p_A=src.last['p_A'],pid_i_A=src.last['i_A'],pid_d_A=src.last['d_A'],
                    joint_drive_torque_Nm=rec.tau_j,belt_actuator_torque_Nm=rec.tau_a,
                    actuator_torque_Nm=rec.tau_m,friction_torque_Nm=rec.tau_f,
                    belt_deflection_rad=rec.theta_s,belt_energy_J=rec.U,
                    belt_stiffness_Nm_per_rad=rec.K_s,actuator_velocity_rad_s=rec.theta_a_dot,
                    joint_velocity_rad_s=qd,belt_fit_exceeded=int(fit)))
            mujoco.mj_step(self.model,self.data);self.k+=1
            if not np.isfinite(self.data.qpos).all():raise ValueError('Nonfinite MuJoCo state')
            angles=self.angles();self.history.append((self.origin+self.k*self.dt,angles))
            for row in rows:
                row['sim_angle_deg']=angles[row['joint']];self.last[row['joint']]=row;self.on_step(row)

    def accept(self,tick):
        t=tick['reference_time_s']
        self.advance(t)
        for j in JOINTS:
            ref=tick['joints'][j]['reference_deg']
            lo,hi=np.degrees(self.model.joint(j).range)
            if not lo<=ref<=hi:raise ValueError(f'{j} target {ref:.3f} outside runtime range [{lo:.1f},{hi:.1f}]; no target clipping')
            self.sources[j].ref[0]=math.radians(ref)
        self.advance(max(v['time_s'] for v in tick['joints'].values()))
        h=list(self.history);result=[]
        for j in JOINTS:
            r=tick['joints'][j];t=r['time_s']
            angle=float(np.interp(t,[x[0] for x in h],[x[1][j] for x in h]))
            hw=r['actual_deg'] if r.get('encoder_valid',True) else None
            result.append(dict(joint=j,source_time_s=t,reference_time_s=tick['reference_time_s'],
                target_deg=r['reference_deg'],hardware_angle_deg=hw,sim_angle_deg=angle,
                difference_deg=angle-hw if hw is not None else None,
                hardware_tracking_error_deg=r['reference_deg']-hw if hw is not None else None,
                sim_tracking_error_deg=r['reference_deg']-angle))
        return result
