import unittest
import numpy as np
from oslbench.sim.assembled_tracking import AssembledTracking
from oslbench.apps.assembled_tracking import archive_ticks
from pathlib import Path

def tick(k,hw=0.,target=0.):
    return dict(reference_time_s=.005+k*.005,joints={j:dict(time_s=.006+k*.005,reference_deg=target,actual_deg=hw,encoder_valid=True) for j in ('knee','ankle')})

class AssembledTests(unittest.TestCase):
    def test_two_independent_negative_feedback_sources(self):
        sim=AssembledTracking(tick(0),feedback='actuator-difference')
        self.assertIsNot(sim.layers['knee'],sim.layers['ankle'])
        for j in ('knee','ankle'):self.assertTrue(sim.sources[j].cfg.negative_feedback)
        sim.accept(tick(0));sim.accept(tick(1,target=1.))
        self.assertTrue(np.all(sim.model.actuator_gainprm==0))
        self.assertTrue(np.all(sim.data.actuator_force==0))
        self.assertGreater(sim.sources['knee'].last['current_request_A'],0)
        self.assertGreater(sim.sources['ankle'].last['current_request_A'],0)

    def test_hardware_angles_do_not_drive_simulation_after_init(self):
        output=[]
        for hw in (0.,8.):
            sim=AssembledTracking(tick(0),feedback='actuator-difference')
            rows=[]
            for k in range(20):rows.extend(sim.accept(tick(k,hw=0. if k==0 else hw,target=1.)))
            output.append([r['sim_angle_deg'] for r in rows])
        np.testing.assert_array_equal(*output)

    def test_range_override_is_explicit(self):
        sim=AssembledTracking(tick(0));bad=tick(1);bad['joints']['ankle']['reference_deg']=25
        with self.assertRaisesRegex(ValueError,'outside runtime range'):sim.accept(bad)
        expanded=AssembledTracking(tick(0),ankle_range='hardware')
        np.testing.assert_allclose(np.degrees(expanded.model.joint('ankle').range),[-31,31])
        self.assertEqual(expanded.metadata['ankle_range_choice'],'hardware')
        invalid=tick(0);invalid['joints']['ankle']['encoder_valid']=False
        with self.assertRaisesRegex(ValueError,'valid encoder'):AssembledTracking(invalid)

    def test_csv_timing_and_channels(self):
        rows=archive_ticks(Path.home()/'Downloads/Tracking Results & Codes & Dataset.zip','knee_ankle_normal_walk_0.6ms')
        row=next(rows)
        self.assertEqual(set(row['joints']),{'knee','ankle'})
        self.assertGreater(row['joints']['ankle']['time_s'],row['joints']['knee']['time_s'])
        self.assertGreater(row['joints']['knee']['time_s'],row['reference_time_s'])

if __name__=='__main__':unittest.main()
