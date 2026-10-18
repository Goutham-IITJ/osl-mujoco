"""Ankle model calibration and MuJoCo current-actuation wiring."""
from __future__ import annotations
import math
import xml.etree.ElementTree as ET
from oslbench.control.current import *
from oslbench.control.servo import PDController
from oslbench.control.compliance import DrivetrainLayer


def derive_mjcf_q_to_encoder_sign(bench, probe_deg: float = 15.0, verbose: bool = False):
    """MEASURE mapping 1 from the compiled model.  Mapping 2 is never consulted.

    Rotates the ankle to +/-probe_deg and watches representative forefoot and heel
    vertices from the actual Vari-Flex mesh.  Geom-frame origins are not usable points:
    MuJoCo recenters mesh assets during compilation, and a geom origin can be near the
    centre of a long mesh.  The compiled vertex array already contains MuJoCo's mesh
    preprocessing offset; applying the geom pose and the body's forward-kinematics
    transform, but not applying ``mesh_pos``/``mesh_quat`` a second time, gives actual
    points on the foot.  Requires the TOE and the HEEL to move OPPOSITE ways -- that is
    what a hinge must do, and demanding both is what makes this a check rather than a
    single-point guess.  Anterior is taken as the direction the foot is LONGER in,
    which is self-establishing from the geometry.

    Returns (sign, report).  `sign` is +1 if positive MuJoCo ankle q matches the
    encoder's reported positive (plantarflexion, toe DOWN), -1 if they oppose, and None
    when the geometry cannot be read -- which is the case on the test stub, since it
    carries no meshes or geoms. Never raises: the caller must be able to fall back to
    the documented default and say that it did.
    """
    try:
        import mujoco
        import numpy as np
        model, data = bench.model, bench.data
        foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "foot")
        if foot < 0:
            return None, "no body named 'foot' in the compiled model"
        geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == foot]
        mesh_geom = None
        for g in geoms:
            mesh_id = int(model.geom_dataid[g])
            mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) or ""
            if "lowprofilevariflexfoot_ossur" in mesh_name.lower():
                mesh_geom = g
                break
        if mesh_geom is None:
            return None, ("the 'foot' body has no compiled "
                          "z_lowprofilevariflexfoot_ossur mesh")

        mesh_id = int(model.geom_dataid[mesh_geom])
        vert_start = int(model.mesh_vertadr[mesh_id])
        vert_count = int(model.mesh_vertnum[mesh_id])
        compiled_vertices = np.asarray(
            model.mesh_vert[vert_start:vert_start + vert_count], dtype=float)
        geom_rot = np.empty(9)
        mujoco.mju_quat2Mat(geom_rot, model.geom_quat[mesh_geom])
        geom_rot = geom_rot.reshape(3, 3)
        # MuJoCo's compiled vertices are already in the post-processed mesh frame.
        # mesh_pos/mesh_quat are recorded below as evidence of that preprocessing, not
        # applied again (doing so double-counts the centering transform).
        foot_local_vertices = ((geom_rot @ compiled_vertices.T).T
                               + np.asarray(model.geom_pos[mesh_geom], dtype=float))

        def sample(angle_deg):
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[bench.ankle_qpos] = math.radians(angle_deg)
            mujoco.mj_forward(model, data)
            body_rot = np.asarray(data.xmat[foot], dtype=float).reshape(3, 3)
            body_pos = np.asarray(data.xpos[foot], dtype=float)
            return (body_rot @ foot_local_vertices.T).T + body_pos

        at0 = sample(0.0)
        # anterior = the direction the foot is longer in, decided at the neutral pose
        i_ant, i_post = int(np.argmin(at0[:, 0])), int(np.argmax(at0[:, 0]))
        span_x = float(at0[:, 0].max() - at0[:, 0].min())
        span_z = float(at0[:, 2].max() - at0[:, 2].min())
        if span_x <= span_z:
            return None, (f"the foot mesh's X extent ({span_x * 1e3:.0f} mm) does not exceed "
                          f"its Z extent ({span_z * 1e3:.0f} mm), so the anterior "
                          f"direction cannot be read from the geometry")
        pos, neg = sample(+probe_deg), sample(-probe_deg)
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        dz_ant = float(pos[i_ant, 2] - neg[i_ant, 2])       # toe height change
        dz_post = float(pos[i_post, 2] - neg[i_post, 2])     # heel height change
        if dz_ant * dz_post >= 0.0:
            return None, (f"toe and heel moved the SAME way (dz {dz_ant * 1e3:+.2f} / "
                          f"{dz_post * 1e3:+.2f} mm); that is not a hinge rotation and "
                          f"the sign cannot be trusted")
        # positive q lowers the toe -> positive q IS plantarflexion -> sign = +1
        sign = 1 if dz_ant < 0.0 else -1
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        asset_file = "unavailable"
        try:
            root = ET.parse(bench.path).getroot()
            for asset in root.findall("./asset/mesh"):
                if asset.get("name") == mesh_name:
                    asset_file = asset.get("file", asset_file)
                    break
        except (ET.ParseError, OSError):
            pass
        report = (f"MEASURED from compiled mesh {mesh_name!r} asset {asset_file!r}: "
                  f"{vert_count} actual vertices, compiled mesh offset "
                  f"pos={np.asarray(model.mesh_pos[mesh_id]).round(6).tolist()} "
                  f"quat={np.asarray(model.mesh_quat[mesh_id]).round(6).tolist()} "
                  f"scale={np.asarray(model.mesh_scale[mesh_id]).round(6).tolist()} "
                  f"(offset already represented in compiled vertices); authored geom "
                  f"pos={np.asarray(model.geom_pos[mesh_geom]).round(6).tolist()} "
                  f"quat={np.asarray(model.geom_quat[mesh_geom]).round(6).tolist()}. "
                  f"Transformed foot-vertex X extent {span_x * 1e3:.0f} mm > Z extent "
                  f"{span_z * 1e3:.0f} mm, so anterior is the -X end; going "
                  f"{-probe_deg:+g} -> {probe_deg:+g} deg of ankle coordinate the toe "
                  f"moved {dz_ant * 1e3:+.2f} mm and the heel {dz_post * 1e3:+.2f} mm "
                  f"(opposite, as a hinge requires), so positive q = "
                  f"{'PLANTARflexion' if dz_ant < 0 else 'DORSIflexion'} and "
                  f"mjcf_q_to_encoder_sign = {sign:+d}")
        if verbose:
            print("  " + report)
        return sign, report
    except Exception as exc:                                    # noqa: BLE001
        return None, (f"could not derive mapping 1 from geometry "
                      f"({type(exc).__name__}: {exc}); expected on the test stub, "
                      f"which carries no meshes")


derive_encoder_sign = derive_mjcf_q_to_encoder_sign


class AnkleCurrentSim:
    """The ankle joint, driven by a commanded current through the Option-C drivetrain.

    WHY THIS IS NOT `DrivetrainBenchSimulation`
        That class is hard-wired to the knee: it disconnects `bench.knee_act`, reads
        `bench.knee_qpos`/`knee_dof`, and writes `data.qfrc_applied[bench.knee_dof]`.
        Pointing it at the ankle would mean editing it, which would put the knee
        experiments and the oracle at risk for an ankle feature. So this is a separate,
        small wiring class that composes the SAME `DrivetrainLayer` -- the belt and
        actuator equations are Best et al.'s, unmodified, and are not restated here.

    WHAT IT DOES PER PHYSICS STEP
        1. read theta_j, theta_j_dot from MuJoCo (pre-step, as the layer expects)
        2. advance the layer with the held current -> belt torque tau_j
        3. write tau_j into qfrc_applied at the ANKLE dof
        4. mj_step

    THE SERVO
        The ankle position actuator is disconnected at construction with the existing
        `PDController(0, 0).write_to_model` trick, so the belt is the only actuation on
        that joint. `ankle_servo_force()` must read exactly 0.0 afterwards -- and it is
        only meaningful AFTER a step, because it reads `data.actuator_force`.

        The KNEE actuator is left connected and holds the knee at its keyframe, which is
        deliberate: the ankle hangs off the shank, and letting the knee flop would change
        the ankle's gravity load mid-experiment.
    """

    def __init__(self, bench, audit: ConversionAudit, layer: DrivetrainLayer | None = None,
                 hold_knee: bool = True, current_loop: "CurrentLoopModel | None" = None):
        self.bench = bench
        self.audit = audit
        # Defaults to an IDEAL loop, so an existing caller that does not pass one gets
        # exactly the behaviour it had before this class existed.
        self.current_loop = current_loop or CurrentLoopModel(0.0)
        self.layer = layer or DrivetrainLayer(audit.params, enabled=True, substeps=1)
        if self.layer.p is not audit.params and (
                self.layer.p.n_t != audit.params.n_t
                or self.layer.p.k_t != audit.params.k_t):
            raise ValueError("the layer and the conversion audit disagree about the "
                             "drivetrain parameters; that would apply two different "
                             "belt ratios in one run")
        self.last_layer = None
        self.servo_connected = True
        self.hold_knee = bool(hold_knee)
        self.last_current = dict(target_A=0.0, actual_A=0.0, layer_rotor_A=0.0)
        self._disconnect_ankle_servo()

    # ----------------------------------------------------------------- the servo
    def _disconnect_ankle_servo(self) -> None:
        """Zero the ANKLE position actuator in the compiled mjModel.

        Reuses `PDController.write_to_model`, so there is still exactly one place in the
        project that knows MuJoCo's position-actuator algebra. ctrlrange and forcerange
        are deliberately untouched, so the authored ROM and torque authority still bound
        everything.
        """
        b = self.bench
        PDController(0.0, 0.0, b.ankle_ctrlrange, b.ankle_forcerange[1],
                     "ankle_disconnected").write_to_model(b.model, b.ankle_act)
        self.servo_connected = False

    def ankle_servo_force(self) -> float:
        """The ankle position actuator's force at the current state, N.m.

        Only meaningful after `mj_forward`/`mj_step`; before that it holds whatever the
        model-loading preflight left in the buffer, from BEFORE the disconnect.
        """
        return float(self.bench.data.actuator_force[self.bench.ankle_act])

    def knee_servo_force(self) -> float:
        """The knee actuator's force -- expected NONZERO: it is holding the knee."""
        return float(self.bench.data.actuator_force[self.bench.knee_act])

    # --------------------------------------------------------------------- state
    @property
    def theta_j(self) -> float:
        return float(self.bench.data.qpos[self.bench.ankle_qpos])

    @property
    def theta_j_dot(self) -> float:
        return float(self.bench.data.qvel[self.bench.ankle_dof])

    def reset(self, q0_rad: float = 0.0, knee_hold_rad: float | None = None) -> None:
        """Put the ankle at its baseline and seed the belt to zero deflection there.

        Seeding rigid (theta_s = 0) means the run starts with no stored belt energy, so
        the first thing the step excites is the controller, not a pre-loaded spring.
        """
        import mujoco
        b = self.bench
        mujoco.mj_resetDataKeyframe(b.model, b.data, 0)
        b.data.qpos[b.ankle_qpos] = float(q0_rad)
        b.data.qvel[:] = 0.0
        b.data.qfrc_applied[:] = 0.0
        if self.hold_knee:
            # command the knee to wherever the keyframe put it, so it is held rather
            # than driven somewhere new
            b.data.ctrl[b.knee_act] = float(b.data.qpos[b.knee_qpos])
        b.data.ctrl[b.ankle_act] = 0.0          # dead actuator; set anyway, not assumed
        mujoco.mj_forward(b.model, b.data)
        self.layer.seed_rigid(float(q0_rad), 0.0)
        self.layer.steps = 0
        self.last_layer = None
        self.current_loop.reset()
        self.last_current = dict(target_A=0.0, actual_A=0.0, layer_rotor_A=0.0)

    # ---------------------------------------------------------------------- step
    def step(self, i_q_output_frame: float):
        """Advance one physics step with a held current command.

        `i_q_output_frame` is the already-signed, already-clipped command, in amps as
        the CONTROLLER means them -- i.e. amps against the chosen `k_t_output`.

        THREE CURRENTS, IN ORDER, AND THE DRIVETRAIN GETS THE LAST ONE
            target        the held command, as handed in
            actual        after `CurrentLoopModel` (identical to target when tau_i = 0)
            layer_rotor   `actual` rescaled into the rotor-amp convention the layer's
                          eq (1) expects, which is what stops n_a being applied twice

        The ACTUAL current is what reaches the drivetrain. Filtering a current for the
        plot while driving the plant with the unfiltered command would make the figure
        a decoration rather than a record, so the single `i_actual` below feeds both.
        """
        import mujoco
        b = self.bench
        dt = float(b.model.opt.timestep)
        theta_j = self.theta_j
        i_target = float(i_q_output_frame)
        i_actual = self.current_loop.update(i_target, dt)
        i_q_layer = self.audit.layer_current_for(i_actual)
        self.last_current = dict(target_A=i_target, actual_A=i_actual,
                                 layer_rotor_A=i_q_layer)
        rec = self.layer.advance(theta_j, i_q_layer, dt)
        b.data.qfrc_applied[b.ankle_dof] = rec.tau_j_mean
        if self.hold_knee:
            b.data.ctrl[b.knee_act] = float(b.data.ctrl[b.knee_act])
        mujoco.mj_step(b.model, b.data)
        self.last_layer = rec
        return rec
