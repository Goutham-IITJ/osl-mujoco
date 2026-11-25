"""Build a standalone, auditable handoff from the supplied hardware ZIP; never run it."""
import ast,hashlib,json,re,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'hardware/gait_tracking'
ARCHIVE=Path.home()/'Downloads/Tracking Results & Codes & Dataset.zip'

def block(name,body,indent=''):
    return f'{indent}# BEGIN TRACKING TELEMETRY: {name}\n'+''.join(indent+l+'\n' for l in body.splitlines())+f'{indent}# END TRACKING TELEMETRY: {name}\n'

def strip_blocks(s):
    return re.sub(r'(?m)^ *# BEGIN TRACKING TELEMETRY: ([^\n]+)\n.*?^ *# END TRACKING TELEMETRY: \1\n','',s,flags=re.S)

def module_source():
    s=(ROOT/'hardware/knee_telemetry.py').read_text()
    s=s[:s.index('\ndef main():')]
    s=s.replace('osl-knee-telemetry-v1','osl-joint-telemetry-v1').replace('KneeTelemetryServer','TrackingTelemetryServer')
    s=s.replace('capacity=4000, source="physical", angle_sign=-1','capacity=8000, source="physical", angle_sign=-1')
    s=s.replace('self.metadata = dict(', 'self.metadata = dict(')
    at='        owner = self\n'
    extra='''        self.metadata.update(joints={"knee": {"angle_multiplier": -1, "current_multiplier": 1},
                                     "ankle": {"angle_multiplier": 1, "current_multiplier": -1}},
                             producer_complete=False,
                             reference_clock="reference_time_s is shared loop-start time; time_s is encoder-read time")
        self.metadata.pop("flexion_positive_multiplier", None)
        self.metadata["angle_frame"] = "raw calibrated per-joint encoder, before CSV cosmetic sign"
        self._joint_sequences = {"knee": 0, "ankle": 0}
'''
    s=s.replace(at,extra+at)
    s=s.replace('encoder_valid, status="running"):',
        'encoder_valid, joint, control_tick, reference_time_s, reference_velocity_deg_s,\n                p_term_A, i_term_A, d_term_A, status="running"):')
    s=s.replace('        sequence = self._sequence\n',
        '        if joint not in self._joint_sequences:\n            raise ValueError("Unknown joint")\n        joint_sequence = self._joint_sequences[joint]\n        self._joint_sequences[joint] += 1\n        sequence = self._sequence\n')
    s=s.replace('encoder_valid=encoder_valid, status=status)',
        'encoder_valid=encoder_valid, status=status, joint=joint, joint_sequence=joint_sequence,\n                   control_tick=control_tick, reference_time_s=reference_time_s,\n                   reference_velocity_deg_s=reference_velocity_deg_s, p_term_A=p_term_A, i_term_A=i_term_A, d_term_A=d_term_A)')
    # Add explicit validation; no change to original module or knee v1 protocol.
    anchor='    if not isinstance(row.get("status"), str)'
    index=s.index(anchor)
    validation='''    if row.get("joint") not in ("knee", "ankle"):
        raise ValueError("invalid joint")
    for key in ("joint_sequence", "control_tick"):
        if type(row.get(key)) is not int or row[key] < 0:
            raise ValueError("invalid " + key)
    for key in ("reference_time_s", "reference_velocity_deg_s", "p_term_A", "i_term_A", "d_term_A"):
        if type(row.get(key)) not in (int, float) or not math.isfinite(row[key]):
            raise ValueError("invalid " + key)
    if not 0 <= row["reference_time_s"] <= row["time_s"]:
        raise ValueError("invalid reference time")
'''
    s=s[:index]+validation+s[index:]
    s+='''
    def finish(self):
        """After motor cleanup, allow the laptop a bounded final drain interval."""
        self.metadata["final_sequence"] = self._sequence - 1
        self.metadata["producer_complete"] = True
        time.sleep(2.)
        self.close()
'''
    s+='''
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Synthetic two-joint telemetry only; no motor access")
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--port", type=int, default=8794)
    args = parser.parse_args()
    server = TrackingTelemetryServer("127.0.0.1", args.port, source="synthetic",
                                    token=os.environ.get("OSL_TELEMETRY_TOKEN"))
    start=time.perf_counter()
    tick=0
    try:
        while time.perf_counter()-start < args.seconds:
            t=time.perf_counter()-start
            for joint in ("knee", "ankle"):
                server.publish(joint=joint,control_tick=tick,reference_time_s=t,time_s=time.perf_counter()-start,
                    dt_s=.005,reference_deg=10*math.sin(t),actual_deg=9*math.sin(t-.05),
                    reference_velocity_deg_s=10*math.cos(t),p_term_A=0.,i_term_A=0.,d_term_A=0.,
                    requested_current_A=0.,commanded_current_A=0.,encoder_valid=True)
            tick+=1
            time.sleep(max(0.,start+tick*.005-time.perf_counter()))
    finally:
        server.finish()
'''
    return s

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=dict(archive_sha256=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),controllers=[])
    (OUT/'telemetry.py').write_text(module_source(),encoding='utf-8')
    with zipfile.ZipFile(ARCHIVE) as z:
        for name in z.namelist():
            if '/Dataset/' in name and name.endswith('.csv'):
                rel=name.split('/Dataset/',1)[1]
                if '..' in Path(rel).parts:raise ValueError('Unsafe path')
                dest=OUT/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(z.read(name))
            if '/Codes/' not in name or not name.endswith('.py'):continue
            original=z.read(name).decode('utf-8-sig');s=original
            # Portable dataset paths are the sole non-telemetry edits.
            if 'normal_walk.py' in name:
                s=s.replace('ANGLE_CSV_PATH = "/home/legpi/test_codes/normal_walk_1-2/AB01_normal_walk_1_1-2_angle.csv"',
                    'ANGLE_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "normal_walk_1-2", "AB01_normal_walk_1_1-2_angle.csv")')
                s=s.replace('GRF_CSV_PATH = "/home/legpi/test_codes/normal_walk_1-2/AB01_normal_walk_1_1-2_grf.csv"',
                    'GRF_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "normal_walk_1-2", "AB01_normal_walk_1_1-2_grf.csv")')
            expected=s
            def insert(anchor,addition):
                nonlocal s
                if s.count(anchor)!=1:raise ValueError('Ambiguous anchor '+anchor)
                s=s.replace(anchor,addition+anchor)
            insert('actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)\n',block('startup',
                'from telemetry import TrackingTelemetryServer\ntelemetry = TrackingTelemetryServer(host="0.0.0.0", port=8794,\n    token=os.environ["OSL_TELEMETRY_TOKEN"], source="physical")\nprint("[telemetry] Both joints: HTTP port 8794; producer starts after homing/hold.")'))
            insert('            joint.log["t"].append(t_wall)\n',block('publish',
                'telemetry.publish(\n    joint=joint.label.lower(), control_tick=i, reference_time_s=t_wall,\n    time_s=now_t-t0_pid, dt_s=dt, reference_deg=reference_deg,\n    reference_velocity_deg_s=vd_degs, actual_deg=actual_deg,\n    requested_current_A=current_cmd, commanded_current_A=current_clamped,\n    p_term_A=p_term, i_term_A=i_term, d_term_A=d_term,\n    encoder_valid=(not jump_bad and joint.i2c_fail_streak == 0),\n    status="abort_pending" if abort_reasons_this_tick else "running",\n)',indent='            '))
            insert('    print("[shutdown] Motors limp; verify by hand before approaching.")\n',block('shutdown',
                'telemetry.finish()',indent='    '))
            assert strip_blocks(s)==expected
            ast.parse(s)
            dest=OUT/Path(name).name;dest.write_text(s,encoding='utf-8')
            manifest['controllers'].append(dict(file=dest.name,source_sha256=hashlib.sha256(z.read(name)).hexdigest(),
                modified_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                verification='Removing marked additions restores source exactly, except two portable dataset paths in normal-walk script'))
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Built',OUT)

if __name__=='__main__':main()
