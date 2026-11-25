import ast,importlib.util,json,threading,unittest
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('tracking_telemetry',ROOT/'hardware/gait_tracking/telemetry.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

def row(joint,t,tick=0):
    return dict(joint=joint,control_tick=tick,time_s=t,reference_time_s=t-.0001,dt_s=.005,
        reference_deg=10.,actual_deg=9.,reference_velocity_deg_s=1.,p_term_A=1.,i_term_A=0.,d_term_A=0.,
        requested_current_A=1.,commanded_current_A=1.,encoder_valid=True)

class TrackingPackageTests(unittest.TestCase):
    def test_schema_auth_channels_and_drop_detection(self):
        server=mod.TrackingTelemetryServer(port=0,token='test-only',capacity=2)
        try:
            with self.assertRaises(HTTPError):urlopen(f'http://127.0.0.1:{server.port}/v1/metadata')
            self.assertTrue(server.publish(**row('knee',.005)))
            self.assertTrue(server.publish(**row('ankle',.006)))
            with urlopen(Request(f'http://127.0.0.1:{server.port}/v1/samples?after=-1',headers={'Authorization':'Bearer test-only'})) as response:
                batch=json.load(response)
            self.assertEqual([r['joint'] for r in batch['samples']],['knee','ankle'])
            self.assertEqual([r['sequence'] for r in batch['samples']],[0,1])
            self.assertEqual([r['joint_sequence'] for r in batch['samples']],[0,0])
            server._lock.acquire()
            try:self.assertFalse(server.publish(**row('knee',.01,1)))
            finally:server._lock.release()
            self.assertTrue(server.publish(**row('ankle',.011,1)))
            self.assertEqual(server.batch()['samples'][-1]['sequence'],3)
            self.assertEqual(server.dropped,1)
        finally:server.close()

    def test_controller_patches_preserve_control_source(self):
        spec=importlib.util.spec_from_file_location('builder',ROOT/'tools/build_tracking_telemetry_package.py')
        builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
        import zipfile
        with zipfile.ZipFile(builder.ARCHIVE) as z:
            for name in z.namelist():
                if '/Codes/' not in name or not name.endswith('.py'):continue
                new=(builder.OUT/Path(name).name).read_text()
                stripped=builder.strip_blocks(new)
                old=z.read(name).decode('utf-8-sig')
                if 'normal_walk.py' in name:
                    trees=[]
                    for tree in (ast.parse(stripped),ast.parse(old)):
                        # Only two dataset-path RHS expressions may differ.
                        for node in tree.body:
                            if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('ANGLE_CSV_PATH','GRF_CSV_PATH'):
                                node.value=ast.Constant('portable dataset path')
                        trees.append(ast.dump(tree))
                    self.assertEqual(trees[-2],trees[-1])
                else:self.assertEqual(stripped,old)
                self.assertIn('joint.i2c_fail_streak',new)
                self.assertLess(new.index('actuators.disable_actuators()'),new.index('telemetry.finish()'))
                self.assertEqual(new.count('# BEGIN TRACKING TELEMETRY:'),3)

if __name__=='__main__':unittest.main()
