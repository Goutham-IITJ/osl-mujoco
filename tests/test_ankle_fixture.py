from pathlib import Path
import unittest
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


class AnkleFixtureTests(unittest.TestCase):
    def test_structure_and_inertia(self):
        full=mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_bench.xml'))
        ankle=mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_ankle_fixture_with_foot.xml'))
        self.assertEqual((ankle.nq,ankle.nv,ankle.nu),(1,1,1))
        self.assertEqual(mujoco.mj_name2id(ankle,mujoco.mjtObj.mjOBJ_JOINT,'knee'),-1)
        np.testing.assert_allclose(ankle.body('ankle_output').mass,full.body('foot').mass)
        np.testing.assert_allclose(ankle.body('ankle_output').inertia,full.body('foot').inertia,atol=1e-10)
        np.testing.assert_allclose(ankle.joint('ankle').axis,full.joint('ankle').axis)
        a,b=mujoco.MjData(full),mujoco.MjData(ankle)
        a.qpos[full.joint('ankle').qposadr[0]]=b.qpos[0]=.2
        mujoco.mj_forward(full,a);mujoco.mj_forward(ankle,b)
        fi,oi=full.body('foot').id,ankle.body('ankle_output').id
        np.testing.assert_allclose(a.xmat[fi],b.xmat[oi],atol=1e-10)
        np.testing.assert_allclose(a.xipos[fi]-a.xpos[fi],b.xipos[oi]-b.xpos[oi],atol=1e-10)
        self.assertEqual(ankle.body('ankle_housing_fixed').dofnum[0],0)

    def test_physics_smoke(self):
        model=mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_ankle_fixture_with_foot.xml'))
        data=mujoco.MjData(model);data.qpos[0]=.1
        for _ in range(2000):mujoco.mj_step(model,data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        self.assertAlmostEqual(data.time,1.)

    def test_replay_uses_own_feedback_and_ankle_channels(self):
        from oslbench.sim.ankle_fixture import AnkleFixtureReplay
        trajectories=[]
        for hardware_angle in (1.,9.):
            prepared=AnkleFixtureReplay.prepare()
            self.assertEqual((prepared[0].kp,prepared[0].ki,prepared[0].kd),(2.,0.,.075))
            steps=[]
            rows=[dict(sequence=k,time_s=.005+k*.005,reference_deg=10.,actual_deg=0. if k==0 else hardware_angle,
                       encoder_valid=True,requested_current_A=0.,commanded_current_A=0.) for k in range(40)]
            sim=AnkleFixtureReplay({'flexion_positive_multiplier':1},rows[0],steps.append,prepared=prepared)
            trajectories.append([sim.accept(r)['sim_angle_deg'] for r in rows])
            self.assertTrue(steps)
            self.assertIn('ankle_drive_torque_Nm',steps[-1])
            self.assertNotIn('knee_drive_torque_Nm',steps[-1])
            self.assertTrue(all(r['ankle_servo_force_Nm']==0 for r in steps))
        np.testing.assert_array_equal(*trajectories)


if __name__=='__main__':unittest.main()
