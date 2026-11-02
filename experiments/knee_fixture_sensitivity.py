"""One-at-a-time diagnostics of unmeasured joint terms; never edits model XML."""
import argparse
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from replay_knee_fixture import read_samples, save_csv, metrics
from oslbench.sim.live_knee import LiveKnee


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,default=ROOT/'build/hardware_step_20261030_134549_560')
    parser.add_argument('--outdir',type=Path)
    args=parser.parse_args()
    out=args.outdir or ROOT/'build'/('knee_fixture_sensitivity_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    source=args.recording/'hardware_batches.jsonl'
    samples=read_samples(source)
    metadata=json.loads((args.recording/'metadata.json').read_text())
    xml=ROOT/'models/osl_v2_knee_fixture.xml'
    original=hashlib.sha256(xml.read_bytes()).hexdigest()
    defaults=dict(armature=.01,damping=.3,frictionloss=.4)
    cases=[('baseline',defaults.copy())]
    for key in defaults:
        for factor in (0.,.5,2.):
            values=defaults.copy()
            values[key]*=factor
            cases.append((f'{key}_{factor:g}x',values))
    summary=[]
    for name,values in cases:
        folder=out/name
        folder.mkdir()
        prepared=LiveKnee.prepare(plant='knee-fixture')
        bench=prepared[1]['bench']
        for key,value in values.items():
            getattr(bench.model,'dof_'+key)[bench.knee_dof]=value
        steps=[]
        runtime=LiveKnee(metadata['hardware'],samples[0],steps.append,prepared=prepared)
        pairs=[]
        failure=None
        try:
            for row in samples:
                pairs.append(runtime.accept(row))
        except ValueError as exc:
            failure=str(exc)
        save_csv(folder/'simulation.csv',steps) if steps else None
        save_csv(folder/'comparison.csv',pairs) if pairs else None
        result=dict(case=name,**values)
        if failure:
            result.update(status='failed',reason=failure)
        else:
            result.update(metrics(pairs,steps))
            result['status']='out_of_belt_fit' if result['belt_fit_exceeded_steps'] else 'complete'
            early=[r['difference_deg'] for r in pairs if r['source_time_s']<=.25]
            result['early_rms_difference_deg']=math.sqrt(sum(e*e for e in early)/len(early))
        (folder/'metadata.json').write_text(json.dumps(dict(
            runtime_joint_parameters=values, model_xml_sha256=original,
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            settings=vars(prepared[0]),
            meaning='One-at-a-time diagnostic factors, not measured or recommended parameters',
            initialization='Same first encoder angle, zero velocity, zero belt deflection',
        ),indent=2,default=str)+'\n')
        summary.append(result)
        print(name,result.get('rms_difference_deg'),result['status'])
    assert hashlib.sha256(xml.read_bytes()).hexdigest()==original
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fields=list(dict.fromkeys(k for r in summary for k in r))
    save_csv(out/'summary.csv',[{k:r.get(k) for k in fields} for r in summary])
    print('Saved:',out)


if __name__=='__main__':
    main()
