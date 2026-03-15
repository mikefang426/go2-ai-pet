import os
import time
import math
import sys

try:
    import pybullet as p
    import pybullet_data
except ModuleNotFoundError as exc:
    raise SystemExit(
        "pybullet is required for quad_sim. Install with "
        "`conda install -c conda-forge pybullet` "
        "(or use `pip install -r requirements-sim.txt` on Python 3.11/3.12)."
    ) from exc

try:
    from .sim_controller import SUPPORTED_ENVIRONMENTS, load_environment
except ImportError:
    from sim_controller import SUPPORTED_ENVIRONMENTS, load_environment

# Stable PyBullet quadruped demo (GUI with fallback) + keyboard control
# Controls:
#   W/S: forward/back
#   A/D: rotate left/right
#   Q/E: strafe left/right
#   Space: stop
#   R: reset pose
#   Esc: quit

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def env_flag(name, default=False):
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")

def connect_bullet(prefer_gui=True):
    # On some macOS drivers, forcing --opengl2 can segfault in native code.
    # Keep default GUI path simple and safe.
    # DIRECT mode is opt-in because keyboard control needs GUI.
    force_direct = os.getenv("QUAD_SIM_FORCE_DIRECT", "0") == "1"
    attempts = []
    if prefer_gui and not force_direct:
        attempts.append((p.GUI, None))
    if force_direct:
        attempts.append((p.DIRECT, None))

    for mode, options in attempts:
        try:
            if options is None:
                cid = p.connect(mode)
            else:
                cid = p.connect(mode, options=options)
        except Exception:
            cid = -1

        if cid >= 0 and p.isConnected() != 0:
            if mode == p.DIRECT:
                print("Using DIRECT mode (no GUI window). Keyboard controls are unavailable.")
            return cid, mode

        if cid >= 0:
            try:
                p.disconnect(cid)
            except Exception:
                pass

    if force_direct:
        raise RuntimeError("PyBullet could not connect to a DIRECT physics server.")
    raise RuntimeError(
        "PyBullet GUI connection failed. Keyboard control requires GUI.\n"
        "If you only want headless simulation, run with QUAD_SIM_FORCE_DIRECT=1."
    )

def load_world():
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    plane = p.loadURDF("plane.urdf")

    env_name = os.getenv("QUAD_SIM_ENV", "empty_room").strip().lower() or "empty_room"
    try:
        env_body_ids = load_environment(env_name, p, pybullet_data)
    except Exception as exc:
        valid = ", ".join(SUPPORTED_ENVIRONMENTS)
        raise RuntimeError(f"Failed to load environment '{env_name}'. Valid options: {valid}") from exc

    if env_body_ids:
        print(f"Loaded environment: {env_name} ({len(env_body_ids)} objects)")
    else:
        print(f"Loaded environment: {env_name}")

    start_pos = [0, 0, 0.45]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])

    candidates = [
        "laikago/laikago_toes_zup.urdf",
        "laikago/laikago_toes_zup_lores.urdf",
        "laikago/laikago_toes.urdf",
        "laikago/laikago.urdf",
        "quadruped/quadruped.urdf",
        "minitaur/minitaur.urdf",
    ]

    data_path = pybullet_data.getDataPath()
    robot = -1
    chosen = None
    for urdf in candidates:
        full = os.path.join(data_path, urdf)
        if os.path.exists(full):
            robot = p.loadURDF(urdf, start_pos, start_orn, useFixedBase=False)
            chosen = urdf
            break

    if robot < 0:
        raise RuntimeError(f"Failed to load any quadruped URDF. Checked {candidates} in {data_path}")

    print(f"Loaded robot URDF: {chosen} (id={robot})")
    if "laikago" in chosen and "zup" not in chosen:
        print("[quad_sim] Warning: non-zup Laikago URDF loaded; pose may look rotated.")
    return plane, robot

def main():
    cid, mode = connect_bullet(prefer_gui=True)

    # Physics params
    hz = 240
    dt = 1.0 / hz
    p.setTimeStep(dt)
    env_name = os.getenv("QUAD_SIM_ENV", "empty_room").strip().lower() or "empty_room"
    stairs_mode = env_name == "stairs" or env_flag("QUAD_SIM_STAIRS_MODE", False)
    debug_controls = env_flag("QUAD_SIM_DEBUG", False)
    # The default Laikago asset's visual "nose" points opposite the joint-space
    # forward direction used by the stock model. In flat scenes, interpret W/S
    # in the robot body frame and flip that body-forward axis once here.
    flat_body_forward_sign = -1.0

    # Visual settings (GUI only)
    if mode == p.GUI:
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0, cameraYaw=45, cameraPitch=-25, cameraTargetPosition=[0, 0, 0.4]
            )
        except Exception:
            pass

    def apply_dynamics(plane_id, robot_id):
        plane_friction = 1.4 if stairs_mode else 1.15
        link_friction = 1.2 if stairs_mode else 1.05
        p.changeDynamics(plane_id, -1, lateralFriction=plane_friction)
        base_linear_damping = 0.04 if stairs_mode else 0.08
        base_angular_damping = 0.04 if stairs_mode else 0.14
        p.changeDynamics(robot_id, -1, linearDamping=base_linear_damping, angularDamping=base_angular_damping)
        for j in range(p.getNumJoints(robot_id)):
            p.changeDynamics(
                robot_id,
                j,
                lateralFriction=link_friction,
                linearDamping=base_linear_damping,
                angularDamping=base_angular_damping,
            )

    leg_joints = {}
    stand_targets_by_id = {}
    stand_joint_ids = []
    gait_phase = 0.0
    joint_name_by_id = {}

    def clamp_joint(joint_name, value):
        if "hip_motor_2_chassis_joint" in joint_name:
            return clamp(value, -0.9, 0.9)
        if "upper_leg_2_hip_motor_joint" in joint_name:
            return clamp(value, -1.3, 1.6)
        if "lower_leg_2_upper_leg_joint" in joint_name:
            return clamp(value, -2.6, -0.2)
        return value

    def configure_joint_layout(robot_id):
        joints = {}
        targets = {}
        names = {}
        for j in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, j)
            if info[2] != p.JOINT_REVOLUTE:
                continue
            name = info[1].decode("utf-8")
            names[j] = name
            targets[j] = p.getJointState(robot_id, j)[0]

            if name.startswith(("FR_", "FL_", "RR_", "RL_")):
                leg = name[:2]
                joints.setdefault(leg, {})
                if "hip_motor_2_chassis_joint" in name:
                    joints[leg]["hip"] = j
                elif "upper_leg_2_hip_motor_joint" in name:
                    joints[leg]["upper"] = j
                elif "lower_leg_2_upper_leg_joint" in name:
                    joints[leg]["lower"] = j
        return joints, targets, names

    def build_stand_targets():
        # Explicit neutral "on all fours" pose for Laikago-like kinematics.
        # Works for zup Laikago and is far more stable than raw spawn pose.
        targets = dict(stand_targets_by_id)
        for leg in ("FR", "FL", "RR", "RL"):
            joints = leg_joints.get(leg, {})
            if not all(k in joints for k in ("hip", "upper", "lower")):
                continue
            side = -1.0 if leg in ("FR", "RR") else 1.0
            hip_id = joints["hip"]
            upper_id = joints["upper"]
            lower_id = joints["lower"]

            if stairs_mode:
                # Lower crouch is deliberate on stairs for clearance and traction.
                hip = side * 0.08
                upper = 0.90
                lower = -1.80
            else:
                # Flat-ground stance should sit closer to a neutral stand than a crouch.
                hip = side * 0.04
                upper = 0.64
                lower = -1.26

            targets[hip_id] = clamp_joint(joint_name_by_id[hip_id], hip)
            targets[upper_id] = clamp_joint(joint_name_by_id[upper_id], upper)
            targets[lower_id] = clamp_joint(joint_name_by_id[lower_id], lower)
        return targets

    def apply_joint_targets(robot_id, target_by_id):
        if not target_by_id:
            return
        ids = list(target_by_id.keys())
        vals = [target_by_id[j] for j in ids]
        n = len(ids)
        motor_force = 320.0 if stairs_mode else 120.0
        pos_gain = 0.92 if stairs_mode else 0.56
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=ids,
            controlMode=p.POSITION_CONTROL,
            targetPositions=vals,
            positionGains=[pos_gain] * n,
            forces=[motor_force] * n,
        )

    def hold_stand_pose(robot_id):
        apply_joint_targets(robot_id, stand_targets_by_id)

    def body_exists(body_id):
        if not isinstance(body_id, int) or body_id < 0:
            return False
        try:
            for i in range(p.getNumBodies()):
                if p.getBodyUniqueId(i) == body_id:
                    return True
        except p.error:
            return False
        return False

    def reload_world():
        nonlocal leg_joints, stand_targets_by_id, stand_joint_ids, gait_phase, joint_name_by_id
        try:
            p.setTimeStep(dt)
            new_plane, new_robot = load_world()
            apply_dynamics(new_plane, new_robot)
            leg_joints, stand_targets_by_id, joint_name_by_id = configure_joint_layout(new_robot)
            stand_targets_by_id = build_stand_targets()
            stand_joint_ids = list(stand_targets_by_id.keys())
            gait_phase = 0.0
            hold_stand_pose(new_robot)
            return new_plane, new_robot
        except Exception:
            return None, None

    def reconnect_and_reload():
        nonlocal cid, mode
        try:
            if p.isConnected() != 0:
                p.disconnect()
        except Exception:
            pass

        # Try to recover GUI first; controls depend on it.
        cid, mode = connect_bullet(prefer_gui=True)
        new_plane, new_robot = reload_world()
        if mode == p.DIRECT:
            print("[quad_sim] Reconnected in DIRECT mode.")
        return new_plane, new_robot

    plane, robot = reload_world()
    if robot is None:
        raise RuntimeError("Failed to initialize simulation world.")

    # Base command (vx, vy, wz)
    vx = vy = wz = 0.0
    target_vx = target_vy = target_wz = 0.0

    max_v = 0.20
    max_wz = 0.8
    gain = 0.12  # smoothing
    if stairs_mode:
        max_v = 0.28
        gain = 0.045

    def key_is_down(keys, ch):
        lo = ord(ch.lower())
        hi = ord(ch.upper())
        return ((lo in keys) and (keys[lo] & p.KEY_IS_DOWN)) or ((hi in keys) and (keys[hi] & p.KEY_IS_DOWN))

    def key_was_triggered(keys, ch):
        lo = ord(ch.lower())
        hi = ord(ch.upper())
        return ((lo in keys) and (keys[lo] & p.KEY_WAS_TRIGGERED)) or ((hi in keys) and (keys[hi] & p.KEY_WAS_TRIGGERED))

    def reset_robot():
        nonlocal vx, vy, wz, target_vx, target_vy, target_wz, plane, robot
        vx = vy = wz = 0.0
        target_vx = target_vy = target_wz = 0.0
        if not body_exists(robot):
            new_plane, new_robot = reload_world()
            if new_robot is not None:
                plane, robot = new_plane, new_robot
            return
        try:
            start_pos = [0, 0, 0.45]
            start_orn = p.getQuaternionFromEuler([0, 0, 0])
            p.resetBasePositionAndOrientation(robot, start_pos, start_orn)
            p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
            hold_stand_pose(robot)
        except p.error:
            new_plane, new_robot = reload_world()
            if new_robot is not None:
                plane, robot = new_plane, new_robot

    reset_robot()

    def apply_base_cmd(vx_cmd, vy_cmd, wz_cmd):
        nonlocal gait_phase
        if not body_exists(robot):
            return False
        try:
            pos, orn = p.getBasePositionAndOrientation(robot)
            roll, pitch, yaw = p.getEulerFromQuaternion(orn)
            lin_vel, ang_vel = p.getBaseVelocity(robot)

            # Only recover if it fully tips; do not overwrite pose every frame.
            if abs(roll) > 0.8 or abs(pitch) > 0.8 or pos[2] < 0.18:
                p.resetBasePositionAndOrientation(
                    robot, [pos[0], pos[1], 0.45], p.getQuaternionFromEuler([0.0, 0.0, yaw])
                )
                p.resetBaseVelocity(robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                hold_stand_pose(robot)
                return True

            # Open-loop trot gait in joint space with conservative amplitudes.
            vx_n = clamp(vx_cmd / max_v, -1.0, 1.0)
            vy_n = clamp(vy_cmd / max_v, -1.0, 1.0)
            wz_n = clamp(wz_cmd / max_wz, -1.0, 1.0)
            cmd_level = max(abs(vx_n), abs(vy_n), abs(wz_n))
            # Keep stair-specific control strictly scoped to the stairs scene.
            in_stairs_env = env_name == "stairs"
            stair_guidance_active = in_stairs_env and stairs_mode and vx_n > 0.0 and -0.25 < pos[0] < 4.8
            on_stair_band = in_stairs_env and 0.35 < pos[0] < 4.6
            climbing_active = in_stairs_env and stairs_mode and vx_n > 0.0 and on_stair_band
            vy_eff = vy_n
            wz_eff = wz_n
            vx_eff = vx_n
            flat_mode = not climbing_active

            if stair_guidance_active:
                # Keep the body aligned with the stair centerline (y ~= 0, yaw ~= 0).
                y_err = -pos[1]
                yaw_err = math.atan2(math.sin(-yaw), math.cos(-yaw))
                vy_track = clamp(1.65 * y_err, -0.32, 0.32)
                wz_track = clamp(2.9 * yaw_err, -1.00, 1.00)
                vy_eff = clamp(vy_track / max_v, -1.0, 1.0)
                wz_eff = clamp(wz_track / max_wz, -1.0, 1.0)
                # Reduce forward progress until centered and facing the stairs.
                align_y = clamp(1.0 - abs(y_err) / 0.65, 0.0, 1.0)
                align_yaw = clamp(1.0 - abs(yaw_err) / 0.75, 0.0, 1.0)
                align = align_y * align_yaw
                vx_eff = vx_n * (0.45 + 0.55 * align)
                climbing_active = climbing_active and align > 0.28

                if abs(pos[1]) > 0.45:
                    p.applyExternalForce(
                        robot,
                        -1,
                        [0.0, 260.0 * y_err, 0.0],
                        [0.0, 0.0, 0.0],
                        p.WORLD_FRAME,
                    )
                if abs(yaw_err) > 0.30:
                    p.applyExternalTorque(
                        robot,
                        -1,
                        [0.0, 0.0, 62.0 * yaw_err],
                        p.WORLD_FRAME,
                    )

            if cmd_level < 0.02:
                gait_phase = 0.0
                hold_stand_pose(robot)
                return True

            if climbing_active:
                cadence_hz = 0.70 + 0.60 * cmd_level
            else:
                cadence_hz = 0.72 + 0.32 * cmd_level
            gait_phase = (gait_phase + 2.0 * math.pi * cadence_hz * dt) % (2.0 * math.pi)

            if climbing_active:
                # Terrain-free gait needs explicit COM/pitch support on stairs.
                progress = clamp((pos[0] - 0.45) / (4.4 - 0.45), 0.0, 1.0)
                assist_x = (55.0 + 26.0 * progress) * vx_eff
                assist_z = (26.0 + 14.0 * progress) * vx_eff
                # Extra boost at the first riser prevents getting stuck at step lip.
                if 0.45 < pos[0] < 1.15 and pos[2] < 0.56:
                    assist_x += 26.0 * vx_eff
                    assist_z += 22.0 * vx_eff
                p.applyExternalForce(robot, -1, [assist_x, 0.0, assist_z], [0.0, 0.0, 0.0], p.WORLD_FRAME)
                target_pitch = 0.16 + 0.10 * progress
                pitch_err = target_pitch - pitch
                pitch_rate = ang_vel[1]
                torque_y = clamp(185.0 * pitch_err - 24.0 * pitch_rate, -75.0, 75.0)
                p.applyExternalTorque(robot, -1, [0.0, torque_y, 0.0], p.WORLD_FRAME)
            elif flat_mode and cmd_level > 0.05:
                # Flat-ground walking should not fight itself. Use external force
                # and stabilizing torques, but avoid hard base-velocity resets.
                body_fwd = flat_body_forward_sign * vx_eff
                body_lat = vy_eff
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                target_lin_x = (body_fwd * cos_yaw - body_lat * sin_yaw) * max_v
                target_lin_y = (body_fwd * sin_yaw + body_lat * cos_yaw) * max_v
                target_yaw_rate = wz_eff * max_wz
                force_x = clamp(140.0 * (target_lin_x - lin_vel[0]), -42.0, 42.0)
                force_y = clamp(110.0 * (target_lin_y - lin_vel[1]), -28.0, 28.0)
                p.applyExternalForce(
                    robot,
                    -1,
                    [force_x, force_y, 0.0],
                    [0.0, 0.0, 0.0],
                    p.WORLD_FRAME,
                )
                roll_torque = clamp(-90.0 * roll - 16.0 * ang_vel[0], -24.0, 24.0)
                pitch_torque = clamp(-105.0 * pitch - 18.0 * ang_vel[1], -28.0, 28.0)
                yaw_torque = clamp(24.0 * (target_yaw_rate - ang_vel[2]), -10.0, 10.0)
                p.applyExternalTorque(
                    robot,
                    -1,
                    [roll_torque, pitch_torque, yaw_torque],
                    p.WORLD_FRAME,
                )

            targets = dict(stand_targets_by_id)
            if flat_mode:
                # Keep flat-ground regulation very mild; strong stance extension caused pogoing.
                target_height = 0.46
                height_err = clamp(target_height - pos[2] - 0.10 * lin_vel[2], -0.06, 0.06)
                pitch_target = 0.0
                pitch_err = clamp(pitch_target - pitch - 0.08 * ang_vel[1], -0.14, 0.14)
            else:
                height_err = 0.0
                pitch_err = 0.0

            for leg in ("FR", "FL", "RR", "RL"):
                joints = leg_joints.get(leg, {})
                if not all(k in joints for k in ("hip", "upper", "lower")):
                    continue

                hip_id = joints["hip"]
                upper_id = joints["upper"]
                lower_id = joints["lower"]

                side = -1.0 if leg in ("FR", "RR") else 1.0
                if climbing_active:
                    # Sequential crawl is more stable than trot on stairs.
                    phase_offset = {
                        "FR": 0.0,
                        "FL": 0.5 * math.pi,
                        "RR": math.pi,
                        "RL": 1.5 * math.pi,
                    }[leg]
                else:
                    phase_offset = 0.0 if leg in ("FR", "RL") else math.pi
                phi = gait_phase + phase_offset
                phase01 = (phi / (2.0 * math.pi)) % 1.0
                swing = math.sin(phi)                   # fore/aft phase
                lift = max(0.0, math.sin(phi))          # swing-leg lift [0..1] in forward half-cycle
                if climbing_active:
                    # Lengthen swing phase to reduce toe strikes at step edges.
                    lift = lift ** 0.62
                elif flat_mode:
                    # Use a longer stance and a shorter, smoother swing on flat ground.
                    duty = 0.60
                    if phase01 < duty:
                        stance_frac = phase01 / duty
                        swing = 1.0 - 2.0 * stance_frac
                        lift = 0.0
                        stance = 1.0
                    else:
                        swing_frac = (phase01 - duty) / (1.0 - duty)
                        swing = -math.cos(math.pi * swing_frac)
                        lift = math.sin(math.pi * swing_frac) ** 1.5
                        stance = 0.0

                # Left turn (wz>0): right legs move forward, left legs backward.
                yaw_leg = -side
                if flat_mode:
                    sweep = 0.12 * (flat_body_forward_sign * vx_eff) + 0.045 * wz_eff * yaw_leg
                else:
                    sweep = 0.22 * vx_eff + 0.16 * wz_eff * yaw_leg
                if climbing_active:
                    sweep *= 1.55

                if flat_mode:
                    hip = stand_targets_by_id[hip_id] + side * (0.025 * vy_eff) + side * (0.006 * swing)
                    upper = stand_targets_by_id[upper_id] + sweep * swing - 0.03 * lift
                    lower = stand_targets_by_id[lower_id] - 0.07 * lift - 0.10 * sweep * swing
                    front_leg = leg in ("FR", "FL")
                    front_sign = -1.0 if front_leg else 1.0
                    # Mild stance support only; avoid kicking the body upward.
                    upper += stance * (0.02 * height_err + 0.006 * front_sign * pitch_err)
                    lower += stance * (0.08 * height_err + 0.02 * front_sign * pitch_err)
                else:
                    hip = stand_targets_by_id[hip_id] + side * (0.10 * vy_eff) + side * (0.02 * swing)
                    upper = stand_targets_by_id[upper_id] + sweep * swing - 0.14 * lift
                    lower = stand_targets_by_id[lower_id] - 0.34 * lift - 0.30 * sweep * swing

                if climbing_active:
                    # Increase toe clearance and pitch body forward while climbing.
                    front_leg = leg in ("FR", "FL")
                    climb_scale = clamp(vx_eff, 0.0, 1.0)
                    leg_lift_scale = 1.32 if front_leg else 1.46
                    upper -= (0.24 * leg_lift_scale) * climb_scale * lift
                    lower -= (0.42 * leg_lift_scale) * climb_scale * lift
                    # Global crouch helps keep contacts loaded on steep transitions.
                    upper += (0.10 if front_leg else 0.07) * climb_scale
                    lower -= 0.16 * climb_scale

                targets[hip_id] = clamp_joint(joint_name_by_id[hip_id], hip)
                targets[upper_id] = clamp_joint(joint_name_by_id[upper_id], upper)
                targets[lower_id] = clamp_joint(joint_name_by_id[lower_id], lower)

            apply_joint_targets(robot, targets)
            return True
        except p.error:
            return False

    print("Controls: W/S forward/back, Q/E strafe, A/D rotate, Space stop, R reset, Esc quit")
    if stairs_mode:
        print("[quad_sim] stairs_mode enabled (higher traction, higher foot lift, climb assist)")
    if debug_controls:
        print("[quad_sim] debug mode enabled (keyboard/velocity telemetry)")

    ESC = 27
    exit_reason = None
    last_debug_log = 0.0

    try:
        while True:
            # If GUI window closed, exit
            if p.isConnected() == 0:
                plane, robot = reconnect_and_reload()
                if robot is None:
                    exit_reason = "physics server disconnected and reconnect failed"
                    break
                reset_robot()
                continue

            if mode == p.GUI:
                try:
                    keys = p.getKeyboardEvents()
                except p.error:
                    plane, robot = reconnect_and_reload()
                    if robot is None:
                        exit_reason = "getKeyboardEvents failed and reconnect failed"
                        break
                    reset_robot()
                    continue
            else:
                keys = {}

            if ESC in keys and (keys[ESC] & p.KEY_WAS_TRIGGERED):
                exit_reason = "ESC pressed"
                break

            if key_was_triggered(keys, 'r'):
                reset_robot()

            if p.B3G_SPACE in keys and (keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED):
                target_vx = target_vy = target_wz = 0.0

            # Held keys map directly to commanded body velocities. This is more
            # reliable than per-frame increments with PyBullet keyboard events.
            w_down = key_is_down(keys, 'w')
            s_down = key_is_down(keys, 's')
            q_down = key_is_down(keys, 'q')
            e_down = key_is_down(keys, 'e')
            a_down = key_is_down(keys, 'a')
            d_down = key_is_down(keys, 'd')

            if w_down == s_down:
                target_vx = 0.0
            else:
                target_vx = max_v if w_down else -max_v

            if q_down == e_down:
                target_vy = 0.0
            else:
                target_vy = max_v if q_down else -max_v

            if a_down == d_down:
                target_wz = 0.0
            else:
                target_wz = max_wz if a_down else -max_wz

            target_vx = clamp(target_vx, -max_v, max_v)
            target_vy = clamp(target_vy, -max_v, max_v)
            target_wz = clamp(target_wz, -max_wz, max_wz)

            vx += (target_vx - vx) * gain
            vy += (target_vy - vy) * gain
            wz += (target_wz - wz) * gain

            if not apply_base_cmd(vx, vy, wz):
                new_plane, new_robot = reload_world()
                if new_robot is None:
                    plane, robot = reconnect_and_reload()
                    if robot is None:
                        exit_reason = "failed to reload world after physics error"
                        break
                else:
                    plane, robot = new_plane, new_robot
                reset_robot()
                continue

            try:
                p.stepSimulation()
            except p.error:
                plane, robot = reconnect_and_reload()
                if robot is None:
                    exit_reason = "stepSimulation failed and reconnect failed"
                    break
                reset_robot()
                continue

            if debug_controls and body_exists(robot):
                now = time.monotonic()
                if now - last_debug_log >= 0.25:
                    try:
                        lin_vel_dbg, ang_vel_dbg = p.getBaseVelocity(robot)
                        print(
                            "[quad_sim] "
                            f"keys(w={int(w_down)} s={int(s_down)} q={int(q_down)} e={int(e_down)} a={int(a_down)} d={int(d_down)}) "
                            f"target(vx={target_vx:.2f} vy={target_vy:.2f} wz={target_wz:.2f}) "
                            f"cmd(vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}) "
                            f"base(vx={lin_vel_dbg[0]:.2f} vy={lin_vel_dbg[1]:.2f} wz={ang_vel_dbg[2]:.2f})"
                        )
                        last_debug_log = now
                    except p.error:
                        pass
            time.sleep(dt)
    finally:
        if exit_reason is not None:
            print(f"[quad_sim] exiting: {exit_reason}")
        try:
            p.disconnect()
        except Exception:
            pass

if __name__ == "__main__":
    main()
