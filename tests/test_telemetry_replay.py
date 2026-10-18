"""Data-only replay fixtures and rejection policies; no physics execution."""
from pathlib import Path
import copy
import csv
import hashlib
import json
import subprocess
import sys
import tempfile

from oslbench.telemetry_replay import ReplaySource, DatasetError, HEADER
from oslbench.contracts import Sample, RobotState

FIXTURE = Path(__file__).parent/'fixtures/telemetry_replay/manifest.json'


def load_fixture():
    m=json.loads(FIXTURE.read_text())
    with FIXTURE.with_name('samples.csv').open(newline='') as f:
        rows=list(csv.DictReader(f))
    return m,rows


def modified(change_manifest=None, change_rows=None):
    m,rows=load_fixture()
    if change_rows:
        change_rows(rows)
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder)
        with (p/'samples.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=HEADER);w.writeheader();w.writerows(rows)
        m['data_sha256']=hashlib.sha256((p/'samples.csv').read_bytes()).hexdigest()
        if change_manifest:
            change_manifest(m)
        (p/'manifest.json').write_text(json.dumps(m))
        return ReplaySource(p/'manifest.json')


def rejects(call, contains):
    try:
        call()
    except DatasetError as exc:
        assert contains in str(exc),str(exc)
    else:
        raise AssertionError('Invalid dataset accepted')


def test_replay_typed_samples_mixed_times_and_provenance():
    r=ReplaySource(FIXTURE)
    assert len(r.records)==9
    state=r.robot_state(0)
    assert isinstance(state,RobotState)
    assert state.knee_angle.time_s==.0005
    assert state.belt_joint_torque.time_s==0
    assert state.knee_angle.unit=='rad' and state.knee_angle.frame=='knee_joint'
    assert state.electromagnetic_output_torque.frame=='actuator_output'
    assert all(isinstance(e.sample,Sample) and e.sample.source=='simulation' for e in r.records[:7])
    assert r.clock_id=='synthetic_elapsed' and r.manifest['identity']['synthetic']
    assert r.records[0].sequence==0 and r.records[0].snapshot_id==0
    assert r.records[0].sample.stage=='step_input'
    # A measured label is preserved as data, never inferred or overwritten as replayed.
    measured=modified(lambda m:m['channels']['knee_angle'].update(source='measured'))
    assert measured.robot_state(0).knee_angle.source=='measured'
    changed=r.manifest;changed['dataset_id']='changed'
    assert r.dataset_id!='changed' and r.manifest['dataset_id']!='changed'


def test_replay_reset_clock_end_and_repeatability():
    r=ReplaySource(FIXTURE)
    assert len(r.advance_to(0))==3
    assert len(r.advance_to(.001))==4
    assert r.advance_to(.002)==() # Explicit gap; nothing synthesized.
    rejects(lambda:r.advance_to(.001),'backwards')
    tail=tuple(r);assert len(tail)==2 and r.ended
    assert r.advance_to(5)==()
    try:next(r)
    except StopIteration:pass
    else:assert False
    r.reset();assert r.time_s==0 and not r.ended
    first=tuple(r);r.reset();assert tuple(r)==first
    assert tuple(ReplaySource(FIXTURE))==first
    empty=modified(lambda m:m.update(record_count=0),lambda rows:rows.clear())
    assert empty.ended and tuple(empty)==()


def test_replay_missing_invalid_and_absent_channels_are_not_zero():
    r=ReplaySource(FIXTURE)
    assert r.records[7].sample is None and r.records[7].time_s==.003
    assert r.records[8].sample.value==1 and not r.records[8].sample.valid
    assert r.robot_state(1) is None and r.robot_state(2) is None
    try:r.robot_state(100)
    except KeyError:pass
    else:assert False
    partial=modified(lambda m:(m.update(record_count=1),m.update(channels={'knee_angle':m['channels']['knee_angle']})),
                     lambda rows:rows.__setitem__(slice(None),[dict(rows[3],sequence='0')]))
    assert len(partial.records)==1 and partial.robot_state(0) is None
    rejects(lambda:modified(change_rows=lambda rows:rows[7].update(valid='1')),'missing value')


def test_replay_schema_units_frames_and_unknown_channel_rejected():
    for key,value in [('schema_version',2),('schema_version',True),('source','hardware-ish'),('record_count',True)]:
        rejects(lambda:modified(lambda m:m.update({key:value})), {'schema_version':'schema_version','source':'provenance','record_count':'record_count'}[key])
    for field,value in [('unit','degree'),('frame','rotor'),('quantity','current')]:
        rejects(lambda:modified(lambda m:m['channels']['knee_angle'].update({field:value})),'quantity/unit/frame')
    rejects(lambda:modified(lambda m:m['clock'].update(unit='ms')),'Clock')
    rejects(lambda:modified(lambda m:m['channels']['knee_angle'].update(stage='unknown')),'stage')
    rejects(lambda:modified(change_rows=lambda rows:rows[0].update(channel='imu')),'undeclared channel')
    rejects(lambda:modified(lambda m:m.update(data_file='../samples.csv')),'sibling')


def test_replay_timestamp_sequence_and_duplicate_group_rejected():
    for key,value,message in [('sequence','0','sequence'),('sequence','20','sequence'),
                              ('time_s','','finite number'),('time_s','-1','timestamp'),
                              ('time_s','nan','non-finite')]:
        rejects(lambda:modified(change_rows=lambda rows:rows[1].update({key:value})),message)
    rejects(lambda:modified(change_rows=lambda rows:rows[7].update(time_s='0')),'timestamp')
    rejects(lambda:modified(change_rows=lambda rows:rows[1].update(channel=rows[0]['channel'])),'duplicate snapshot/channel')


def test_replay_nonfinite_even_invalid_and_malformed_values_rejected():
    for val in ('nan','inf','-inf','abc'):
        rejects(lambda:modified(change_rows=lambda rows:rows[8].update(value=val)), 'finite')
    rejects(lambda:modified(change_rows=lambda rows:rows[0].update(valid='true')),'valid')
    rejects(lambda:modified(change_rows=lambda rows:rows.pop()),'record_count')
    rejects(lambda:modified(lambda m:m.update(data_sha256='0'*64)),'SHA-256')


def test_replay_malformed_json_and_csv_files():
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder);manifest=p/'manifest.json'
        for body in ('{', '[]', '{"schema_version":1,"schema_version":1}'):
            manifest.write_text(body)
            rejects(lambda:ReplaySource(manifest),'Manifest' if body!='{"schema_version":1,"schema_version":1}' else 'Duplicate JSON')
        m,_=load_fixture()
        for data in ('bad,header\n', ','.join(HEADER)+'\n0,0,knee_angle\n', ','.join(HEADER)+'\n"unterminated'):
            (p/'samples.csv').write_text(data)
            m['data_sha256']=hashlib.sha256((p/'samples.csv').read_bytes()).hexdigest()
            manifest.write_text(json.dumps(m))
            rejects(lambda:ReplaySource(manifest),'CSV')


def test_replay_import_and_iteration_need_no_mujoco_or_commands():
    # Fresh interpreter: a finder raises even on an attempted import, not just loading.
    code = """
import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('mujoco','numpy'):
            raise AssertionError('Forbidden replay dependency: '+fullname)
sys.meta_path.insert(0,Block())
from oslbench.telemetry_replay import ReplaySource
from oslbench.contracts import Sample
r=ReplaySource(sys.argv[1])
assert all(e.sample is None or isinstance(e.sample,Sample) for e in r)
assert 'oslbench.simulation' not in sys.modules
assert 'oslbench.controller' not in sys.modules
assert 'mujoco' not in sys.modules
"""
    result=subprocess.run([sys.executable,'-B','-c',code,str(FIXTURE.resolve())],cwd=FIXTURE.parents[3],capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_replay_lazy_package_exports_are_compatible():
    import importlib
    import oslbench
    for module,names in oslbench._EXPORTS.items():
        original=importlib.import_module('oslbench.'+module)
        for name in names:
            assert getattr(oslbench,name) is getattr(original,name)
            assert name in dir(oslbench)
    try:getattr(oslbench,'not_an_export')
    except AttributeError:pass
    else:assert False
