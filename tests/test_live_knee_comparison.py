import unittest
from unittest.mock import patch
from oslbench.sim.live_knee import LiveKnee


def sample(k, actual=-10., target=-12.):
    return dict(sequence=k,time_s=4+k*.005,dt_s=.005,reference_deg=target,
        actual_deg=actual,encoder_valid=True,status='running',
        requested_current_A=0.,commanded_current_A=0.)


class LiveComparisonTests(unittest.TestCase):
    def test_dashboard_axes_pause_and_separate_plots(self):
        import json, shutil, subprocess, tempfile
        from pathlib import Path
        from oslbench.presentation.live_knee import PAGE, GROUPS
        node=shutil.which('node')
        if not node:self.skipTest('Node.js required for executable UI check')
        with tempfile.TemporaryDirectory() as folder:
            for mode in ('comparison','telemetry'):
                html=PAGE.replace('MODE',json.dumps(mode)).replace('OTHER','"http://localhost/"').replace('GROUPS',json.dumps(GROUPS))
                path=Path(folder)/(mode+'.js')
                path.write_text(html.split('<script>')[1].split('</script>')[0],encoding='utf-8')
                result=subprocess.run([node,str(Path(__file__).with_name('live_knee_dashboard_check.js')),str(path),mode],capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_stop_control_is_explicit_and_shared(self):
        from oslbench.presentation.live_knee import Presentation
        p=Presentation({'source':'synthetic','session_id':'test'})
        p.key('space');self.assertFalse(p.stop_requested)
        p.key('stop-recording');self.assertTrue(p.stop_requested)

    def runtime(self, write=lambda r:None):
        with patch('oslbench.sim.current_gait.load_reference',side_effect=AssertionError('No CSV reference allowed')):
            return LiveKnee(dict(flexion_positive_multiplier=-1),sample(0),write)

    def test_independent_feedback_and_same_target(self):
        a,b=self.runtime(),self.runtime()
        for k in range(41):
            ra=a.accept(sample(k,actual=-10.))
            rb=b.accept(sample(k,actual=-50. if k else -10.))
        self.assertAlmostEqual(ra['sim_angle_deg'],rb['sim_angle_deg'],12)
        self.assertNotEqual(ra['difference_deg'],rb['difference_deg'])
        self.assertEqual(a.k,400)
        self.assertEqual(a.src.updates,40)
        self.assertEqual(a.sim.knee_servo_force(),0.)

    def test_time_alignment_and_step_recording(self):
        records=[];runtime=self.runtime(records.append)
        runtime.accept(sample(0))
        runtime.accept(sample(1))
        self.assertEqual(len(records),10)
        self.assertAlmostEqual(records[-1]['source_time_s'],4.005)
        self.assertIn('belt_deflection_rad',records[-1])
        self.assertIn('pid_i_A',records[-1])
        self.assertAlmostEqual(runtime.ctx['dt'],.0005)

    def test_reference_change_is_causal(self):
        records=[];runtime=self.runtime(records.append)
        runtime.accept(sample(0,target=-12))
        runtime.accept(sample(1,target=-15))
        self.assertTrue(all(abs(r['target_deg']-12)<1e-10 for r in records))
        runtime.accept(sample(2,target=-15))
        self.assertAlmostEqual(records[-1]['target_deg'],15.)

    def test_gaps_stop_and_invalid_encoder_excluded(self):
        runtime=self.runtime();runtime.accept(sample(0))
        with self.assertRaisesRegex(ValueError,'gap'):runtime.accept(sample(2))
        row=sample(1);row['encoder_valid']=False
        self.assertIsNone(runtime.accept(row)['difference_deg'])

    def test_fractional_source_time_alignment(self):
        runtime=self.runtime();runtime.accept(sample(0))
        row=sample(1);row['time_s']=4.0052
        result=runtime.accept(row)
        self.assertEqual(runtime.k,11)
        self.assertAlmostEqual(result['source_time_s'],4.0052)
        self.assertAlmostEqual(result['sim_angle_deg'],runtime.previous_q+.4*(runtime.q-runtime.previous_q),9)

    def test_prepared_model_uses_first_sample_without_reloading(self):
        prepared=LiveKnee.prepare()
        first=sample(0,actual=-23.,target=-31.)
        records=[]
        with patch('oslbench.sim.live_knee.build',side_effect=AssertionError('Must not load after connection')):
            runtime=LiveKnee(dict(flexion_positive_multiplier=-1),first,records.append,prepared=prepared)
        self.assertIs(runtime.ctx,prepared[1])
        self.assertAlmostEqual(runtime.q,23.)
        self.assertEqual(runtime.k,0)
        runtime.accept(first)
        runtime.accept(sample(1,actual=-23.,target=-31.))
        self.assertTrue(all(abs(row['target_deg']-31.)<1e-10 for row in records))

    def test_viewer_is_ready_before_connection_and_closed_on_connection_failure(self):
        from types import SimpleNamespace
        from oslbench.apps.live_knee_comparison import main
        import tempfile
        from pathlib import Path
        events=[]
        viewer=SimpleNamespace(sync=lambda:events.append('sync'),close=lambda:events.append('close'))
        ctx={'bench':SimpleNamespace(model=object(),data=object())}
        def launch(*args):events.append('viewer');return viewer
        def connect(*args):
            events.append('connect')
            self.assertEqual(events,['prepare','viewer','sync','connect'])
            raise RuntimeError('test connection failure')
        def prepare():events.append('prepare');return None,ctx
        with tempfile.TemporaryDirectory() as directory, \
             patch('oslbench.apps.live_knee_comparison.LiveKnee.prepare',side_effect=prepare), \
             patch('mujoco.viewer.launch_passive',side_effect=launch), \
             patch('oslbench.apps.live_knee_comparison.connect_receiver',side_effect=connect):
            with self.assertRaisesRegex(RuntimeError,'test connection failure'):
                main(['--view','--outdir',str(Path(directory)/'new')])
        self.assertEqual(events[-1],'close')


if __name__=='__main__':unittest.main()
