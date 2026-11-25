"""Checks for the reversed fixture, independent of full-leg runtime assumptions."""
from pathlib import Path
import unittest
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class KneeFixtureTests(unittest.TestCase):
    def setUp(self):
        self.full = mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_bench.xml'))
        self.fixture = mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_knee_fixture.xml'))

    def test_one_joint_and_no_ankle(self):
        self.assertEqual((self.fixture.nq,self.fixture.nv,self.fixture.nu),(1,1,1))
        self.assertEqual(mujoco.mj_name2id(self.fixture,mujoco.mjtObj.mjOBJ_JOINT,'ankle'),-1)
        self.assertEqual(self.fixture.opt.timestep,self.full.opt.timestep)

    def test_moving_inertia_and_neutral_pose_preserved(self):
        a,b=mujoco.MjData(self.full),mujoco.MjData(self.fixture)
        mujoco.mj_forward(self.full,a)
        mujoco.mj_forward(self.fixture,b)
        i=self.full.body('knee_prox').id
        j=self.fixture.body('knee_prox').id
        self.assertAlmostEqual(self.full.body_mass[i],self.fixture.body_mass[j])
        np.testing.assert_allclose(self.full.body_inertia[i],self.fixture.body_inertia[j])
        np.testing.assert_allclose(a.xipos[i],b.xipos[j],atol=1e-10)
        original=np.flatnonzero(self.full.geom_bodyid==i)
        moved=np.flatnonzero(self.fixture.geom_bodyid==j)
        np.testing.assert_allclose(a.geom_xpos[original],b.geom_xpos[moved],atol=1e-10)
        np.testing.assert_allclose(a.geom_xmat[original],b.geom_xmat[moved],atol=1e-10)

    def test_relative_angle_and_fixed_housing(self):
        a,b=mujoco.MjData(self.full),mujoco.MjData(self.fixture)
        a.qpos[0]=b.qpos[0]=.26
        mujoco.mj_forward(self.full,a)
        mujoco.mj_forward(self.fixture,b)
        lower=self.full.body('shank').id
        upper=self.fixture.body('knee_prox').id
        fixed=self.fixture.body('knee_housing_fixed').id
        # Original lower relative to upper == inverse(new upper relative to lower).
        np.testing.assert_allclose(a.xmat[lower].reshape(3,3),b.xmat[upper].reshape(3,3).T,atol=1e-10)
        np.testing.assert_allclose(b.xmat[fixed].reshape(3,3),np.eye(3),atol=1e-10)
        self.assertEqual(self.fixture.body_dofnum[fixed],0)

    def test_physics_smoke(self):
        data=mujoco.MjData(self.fixture)
        data.qpos[0]=.1
        data.ctrl[0]=.05
        for _ in range(2000):
            mujoco.mj_step(self.fixture,data)
        self.assertTrue(np.isfinite(data.qpos).all())
        self.assertTrue(np.isfinite(data.qvel).all())
        self.assertAlmostEqual(data.time,1.)

    def test_shared_controller_and_independent_feedback(self):
        from oslbench.sim.live_knee import LiveKnee
        from oslbench.control.feedback import ActuatorDifferenceSource
        def samples(angle):
            return [dict(sequence=k, time_s=.005+k*.005, reference_deg=15.,
                         actual_deg=0. if k==0 else angle, encoder_valid=True,
                         requested_current_A=0., commanded_current_A=0.) for k in range(40)]
        trajectories=[]
        for measured in (2., 13.):
            prepared=LiveKnee.prepare(plant='knee-fixture')
            self.assertIsInstance(prepared[1]['src'],ActuatorDifferenceSource)
            self.assertEqual(prepared[0].control_hz,200.)
            self.assertEqual((prepared[0].kp,prepared[0].ki,prepared[0].kd),(1.5,.15,.06))
            rows=samples(measured)
            sim=LiveKnee({'flexion_positive_multiplier':1},rows[0],prepared=prepared)
            trajectories.append([sim.accept(row)['sim_angle_deg'] for row in rows])
            self.assertEqual(sim.sim.knee_servo_force(),0.)
        # Changing recorded angles after initialization must NOT drive the simulation.
        np.testing.assert_array_equal(*trajectories)


if __name__=='__main__':
    unittest.main()
