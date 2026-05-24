import os
import time
import math
import sys
import json

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

def env_sign(name, default=1.0):
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError:
        return default
    return -1.0 if value < 0.0 else 1.0

def env_float(name, default):
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default

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
    # Keyboard controls should match the robot's visible heading. Allow an env
    # override because different quadruped assets may disagree about which body
    # axis is visually "forward".
    flat_body_forward_sign = env_sign("QUAD_SIM_FORWARD_SIGN", 1.0)

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
    sit_targets_by_id = {}
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

    def build_sit_targets():
        # A compact seated pose for simple text-command demos.
        targets = dict(stand_targets_by_id)
        for leg in ("FR", "FL", "RR", "RL"):
            joints = leg_joints.get(leg, {})
            if not all(k in joints for k in ("hip", "upper", "lower")):
                continue
            side = -1.0 if leg in ("FR", "RR") else 1.0
            hip_id = joints["hip"]
            upper_id = joints["upper"]
            lower_id = joints["lower"]
            hip = side * 0.08
            upper = 1.05
            lower = -2.10
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

    def hold_sit_pose(robot_id):
        apply_joint_targets(robot_id, sit_targets_by_id)

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
        nonlocal leg_joints, stand_targets_by_id, sit_targets_by_id, stand_joint_ids, gait_phase, joint_name_by_id
        try:
            p.setTimeStep(dt)
            new_plane, new_robot = load_world()
            apply_dynamics(new_plane, new_robot)
            leg_joints, stand_targets_by_id, joint_name_by_id = configure_joint_layout(new_robot)
            stand_targets_by_id = build_stand_targets()
            sit_targets_by_id = build_sit_targets()
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

    max_v = env_float("QUAD_SIM_MAX_V", 0.30)
    max_wz = env_float("QUAD_SIM_MAX_WZ", 0.8)
    manual_gain = env_float("QUAD_SIM_MANUAL_GAIN", 0.12)
    external_lin_gain = env_float("QUAD_SIM_EXTERNAL_LIN_GAIN", 0.045)
    external_wz_gain = env_float("QUAD_SIM_EXTERNAL_WZ_GAIN", 0.040)
    manual_lin_accel = env_float("QUAD_SIM_MANUAL_LIN_ACCEL", 2.60)
    manual_wz_accel = env_float("QUAD_SIM_MANUAL_WZ_ACCEL", 6.50)
    external_lin_accel = env_float("QUAD_SIM_EXTERNAL_LIN_ACCEL", 0.85)
    external_wz_accel = env_float("QUAD_SIM_EXTERNAL_WZ_ACCEL", 1.80)
    sit_speed_threshold = env_float("QUAD_SIM_SIT_SPEED_THRESHOLD", 0.05)
    sit_turn_threshold = env_float("QUAD_SIM_SIT_TURN_THRESHOLD", 0.16)
    posture_blend_rate = env_float("QUAD_SIM_POSTURE_BLEND_RATE", 2.80)
    posture_blend = 1.0  # 1.0=stand, 0.0=sit
    if stairs_mode:
        max_v = 0.28
        manual_gain = 0.045
        external_lin_gain = min(external_lin_gain, 0.040)
        external_wz_gain = min(external_wz_gain, 0.035)
        external_lin_accel = min(external_lin_accel, 0.70)
        external_wz_accel = min(external_wz_accel, 1.40)
    external_cmd_file = os.getenv("QUAD_SIM_CMD_FILE", "").strip()
    external_posture = "stand"
    external_action = None
    external_action_id = -1
    last_external_action_id = -1
    idle_anchor_x = 0.0
    idle_anchor_y = 0.0
    idle_anchor_yaw = 0.0
    idle_hold_active = False
    if external_cmd_file:
        print(f"[quad_sim] external command mode enabled: {external_cmd_file}")

    def key_is_down(keys, ch):
        lo = ord(ch.lower())
        hi = ord(ch.upper())
        return ((lo in keys) and (keys[lo] & p.KEY_IS_DOWN)) or ((hi in keys) and (keys[hi] & p.KEY_IS_DOWN))

    def key_was_triggered(keys, ch):
        lo = ord(ch.lower())
        hi = ord(ch.upper())
        return ((lo in keys) and (keys[lo] & p.KEY_WAS_TRIGGERED)) or ((hi in keys) and (keys[hi] & p.KEY_WAS_TRIGGERED))

    def read_external_command():
        if not external_cmd_file:
            return None
        try:
            with open(external_cmd_file, "r", encoding="utf-8") as fh:
                payload = json.load(fh)
        except FileNotFoundError:
            return None
        except Exception:
            return None

        try:
            vx_cmd = float(payload.get("vx", 0.0))
            vy_cmd = float(payload.get("vy", 0.0))
            wz_cmd = float(payload.get("wz", 0.0))
        except (TypeError, ValueError):
            return None

        posture = str(payload.get("posture", "stand")).strip().lower()
        if posture not in {"stand", "sit"}:
            posture = "stand"
        action = str(payload.get("action", "")).strip().lower() or None
        if action not in {None, "beg", "greet", "shake_hand"}:
            action = None
        try:
            action_id = int(payload.get("action_id", 0))
        except (TypeError, ValueError):
            action_id = 0

        return {
            "vx": clamp(vx_cmd, -max_v, max_v),
            "vy": clamp(vy_cmd, -max_v, max_v),
            "wz": clamp(wz_cmd, -max_wz, max_wz),
            "posture": posture,
            "action": action,
            "action_id": action_id,
        }

    def smooth_axis(current, target, gain, accel_limit):
        gain = clamp(gain, 0.0, 1.0)
        blended = current + (target - current) * gain
        if accel_limit <= 0.0:
            return blended
        max_delta = accel_limit * dt
        delta = blended - current
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return current + delta

    def blend_static_pose(alpha):
        alpha = clamp(alpha, 0.0, 1.0)
        # Linear blend between sit (0.0) and stand (1.0) for gentler posture transitions.
        targets = {}
        for joint_id, stand_val in stand_targets_by_id.items():
            sit_val = sit_targets_by_id.get(joint_id, stand_val)
            targets[joint_id] = sit_val + (stand_val - sit_val) * alpha
        return targets

    def blend_landing_pose(alpha, nose_up_bias=0.0):
        # Landing pose helper: front legs extend slightly while rear legs
        # compress a bit to resist nose-diving at touchdown.
        targets = blend_static_pose(alpha)
        b = clamp(nose_up_bias, 0.0, 1.0)
        if b <= 1e-6:
            return targets

        for leg in ("FR", "FL", "RR", "RL"):
            joints = leg_joints.get(leg, {})
            if not all(k in joints for k in ("upper", "lower")):
                continue
            upper_id = joints["upper"]
            lower_id = joints["lower"]
            front = leg in ("FR", "FL")
            if front:
                upper = targets[upper_id] - 0.10 * b
                lower = targets[lower_id] - 0.18 * b
            else:
                upper = targets[upper_id] + 0.07 * b
                lower = targets[lower_id] + 0.12 * b
            targets[upper_id] = clamp_joint(joint_name_by_id[upper_id], upper)
            targets[lower_id] = clamp_joint(joint_name_by_id[lower_id], lower)
        return targets

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

    def execute_greet_action():
        nonlocal vx, vy, wz, target_vx, target_vy, target_wz, posture_blend, gait_phase, external_posture
        nonlocal idle_hold_active
        if not body_exists(robot):
            return

        try:
            pos0, orn0 = p.getBasePositionAndOrientation(robot)
            _, _, yaw0 = p.getEulerFromQuaternion(orn0)
        except p.error:
            return

        vx = vy = wz = 0.0
        target_vx = target_vy = target_wz = 0.0
        gait_phase = 0.0
        posture_blend = 1.0
        external_posture = "stand"
        idle_hold_active = False

        def angle_delta(a, b):
            return math.atan2(math.sin(a - b), math.cos(a - b))

        def smoothstep01(t):
            t = clamp(t, 0.0, 1.0)
            return t * t * (3.0 - 2.0 * t)

        def build_greet_targets(bow, wag, bounce, nod):
            targets = dict(stand_targets_by_id)

            for leg in ("FR", "FL", "RR", "RL"):
                joints = leg_joints.get(leg, {})
                if not all(k in joints for k in ("hip", "upper", "lower")):
                    continue
                front_leg = leg in {"FR", "FL"}
                right_leg = leg in {"FR", "RR"}
                side = 1.0 if right_leg else -1.0
                hip_id = joints["hip"]
                upper_id = joints["upper"]
                lower_id = joints["lower"]

                if front_leg:
                    hip_offset = side * 0.05 * wag
                    upper_offset = 0.34 * bow - 0.08 * bounce + 0.08 * nod
                    lower_offset = -0.56 * bow + 0.14 * bounce - 0.12 * nod
                else:
                    hip_offset = -side * 0.10 * wag
                    upper_offset = -0.12 * bow - 0.10 * bounce - 0.04 * nod
                    lower_offset = 0.24 * bow + 0.18 * bounce + 0.06 * nod

                targets[hip_id] = clamp_joint(
                    joint_name_by_id[hip_id],
                    stand_targets_by_id[hip_id] + hip_offset,
                )
                targets[upper_id] = clamp_joint(
                    joint_name_by_id[upper_id],
                    stand_targets_by_id[upper_id] + upper_offset,
                )
                targets[lower_id] = clamp_joint(
                    joint_name_by_id[lower_id],
                    stand_targets_by_id[lower_id] + lower_offset,
                )
            return targets

        def step_greet(bow, wag, bounce, nod):
            if not body_exists(robot):
                return False
            try:
                pos, orn = p.getBasePositionAndOrientation(robot)
                roll, pitch, yaw = p.getEulerFromQuaternion(orn)
                lin_vel, ang_vel = p.getBaseVelocity(robot)
            except p.error:
                return False

            x_err = pos[0] - pos0[0]
            y_err = pos[1] - pos0[1]
            yaw_err = angle_delta(yaw0, yaw)
            support_height = 0.43 - 0.040 * bow + 0.022 * bounce
            fx = clamp(-176.0 * x_err - 40.0 * lin_vel[0], -78.0, 78.0)
            fy = clamp(-188.0 * y_err - 40.0 * lin_vel[1], -88.0, 88.0)
            fz = clamp(46.0 * (support_height - pos[2]) - 54.0 * lin_vel[2], -70.0, 46.0)
            tx = clamp((-194.0 * roll - 36.0 * ang_vel[0]) + 18.0 * wag, -64.0, 64.0)
            ty = clamp((-176.0 * pitch - 32.0 * ang_vel[1]) - 32.0 * bow + 14.0 * nod, -62.0, 62.0)
            tz = clamp(128.0 * yaw_err - 24.0 * ang_vel[2] + 12.0 * wag, -44.0, 44.0)

            try:
                apply_joint_targets(robot, build_greet_targets(bow, wag, bounce, nod))
                p.applyExternalForce(robot, -1, [fx, fy, fz], [0.0, 0.0, 0.0], p.WORLD_FRAME)
                p.applyExternalTorque(robot, -1, [tx, ty, tz], p.WORLD_FRAME)
                p.stepSimulation()
                time.sleep(dt)
                return True
            except p.error:
                return False

        settle_steps = max(1, int(0.22 / dt))
        for _ in range(settle_steps):
            if not step_greet(0.0, 0.0, 0.0, 0.0):
                return

        bow_steps = max(1, int(0.56 / dt))
        for i in range(bow_steps):
            phase = smoothstep01((i + 1) / float(bow_steps))
            bow = math.sin(phase * math.pi) * 0.92
            nod = math.sin(phase * math.pi * 2.0) * 0.38
            if not step_greet(bow, 0.0, 0.0, nod):
                return

        wiggle_steps = max(1, int(1.30 / dt))
        for i in range(wiggle_steps):
            phase = (i + 1) / float(wiggle_steps)
            wag = math.sin(phase * math.pi * 8.0)
            bounce = 0.5 + 0.5 * math.sin(phase * math.pi * 6.0)
            bow = 0.24 + 0.10 * math.sin(phase * math.pi * 2.0)
            nod = math.sin(phase * math.pi * 4.0) * 0.24
            if not step_greet(bow, wag, bounce, nod):
                return

        pop_steps = max(1, int(0.42 / dt))
        for i in range(pop_steps):
            phase = (i + 1) / float(pop_steps)
            bounce = max(0.0, math.sin(phase * math.pi * 3.0))
            nod = math.sin(phase * math.pi * 3.0) * 0.30
            if not step_greet(0.0, 0.0, bounce, nod):
                return

        reset_steps = max(1, int(0.36 / dt))
        for i in range(reset_steps):
            phase = 1.0 - smoothstep01((i + 1) / float(reset_steps))
            if not step_greet(0.18 * phase, 0.0, 0.0, 0.0):
                return

        settle_back_steps = max(1, int(0.26 / dt))
        for _ in range(settle_back_steps):
            if not step_greet(0.0, 0.0, 0.0, 0.0):
                return

    def execute_beg_action():
        nonlocal vx, vy, wz, target_vx, target_vy, target_wz, posture_blend, gait_phase, external_posture
        nonlocal idle_hold_active
        if not body_exists(robot):
            return

        try:
            pos0, orn0 = p.getBasePositionAndOrientation(robot)
            _, pitch0, yaw0 = p.getEulerFromQuaternion(orn0)
        except p.error:
            return

        vx = vy = wz = 0.0
        target_vx = target_vy = target_wz = 0.0
        gait_phase = 0.0
        posture_blend = 1.0
        external_posture = "stand"
        idle_hold_active = False
        toe_link_by_leg = {}
        for joint_id in range(p.getNumJoints(robot)):
            link_name = p.getJointInfo(robot, joint_id)[12].decode("utf-8")
            if link_name in {"toeFR", "toeFL", "toeRR", "toeRL"}:
                toe_link_by_leg[link_name[-2:]] = joint_id

        rear_anchor = None
        if "RR" in toe_link_by_leg and "RL" in toe_link_by_leg:
            try:
                rr_pos = p.getLinkState(robot, toe_link_by_leg["RR"])[0]
                rl_pos = p.getLinkState(robot, toe_link_by_leg["RL"])[0]
                rear_anchor = [
                    (rr_pos[0] + rl_pos[0]) * 0.5,
                    (rr_pos[1] + rl_pos[1]) * 0.5,
                ]
            except p.error:
                rear_anchor = None

        def smoothstep01(t):
            t = clamp(t, 0.0, 1.0)
            return t * t * (3.0 - 2.0 * t)

        def build_beg_targets(rise, paw):
            targets = dict(stand_targets_by_id)

            for leg in ("FR", "FL", "RR", "RL"):
                joints = leg_joints.get(leg, {})
                if not all(k in joints for k in ("hip", "upper", "lower")):
                    continue
                front_leg = leg in {"FR", "FL"}
                side = -1.0 if leg in {"FR", "RR"} else 1.0
                hip_id = joints["hip"]
                upper_id = joints["upper"]
                lower_id = joints["lower"]

                if front_leg:
                    final_hip = side * (0.10 + 0.02 * paw)
                    final_upper = 1.42 + 0.06 * paw
                    final_lower = -2.48 - 0.08 * max(0.0, paw)
                else:
                    final_hip = side * 0.12
                    final_upper = 0.98
                    final_lower = -2.04

                targets[hip_id] = clamp_joint(
                    joint_name_by_id[hip_id],
                    stand_targets_by_id[hip_id] + (final_hip - stand_targets_by_id[hip_id]) * rise,
                )
                targets[upper_id] = clamp_joint(
                    joint_name_by_id[upper_id],
                    stand_targets_by_id[upper_id] + (final_upper - stand_targets_by_id[upper_id]) * rise,
                )
                targets[lower_id] = clamp_joint(
                    joint_name_by_id[lower_id],
                    stand_targets_by_id[lower_id] + (final_lower - stand_targets_by_id[lower_id]) * rise,
                )
            return targets

        def step_beg(rise, paw):
            if not body_exists(robot):
                return False

            try:
                targets = build_beg_targets(rise, paw)
                for joint_id, target in targets.items():
                    p.resetJointState(robot, joint_id, target)

                target_pitch = pitch0 - 0.72 * rise + 0.025 * paw * rise
                target_orn = p.getQuaternionFromEuler([0.0, target_pitch, yaw0])
                target_pos = [pos0[0], pos0[1], pos0[2]]
                p.resetBasePositionAndOrientation(robot, target_pos, target_orn)

                if rear_anchor is not None:
                    rr_pos = p.getLinkState(robot, toe_link_by_leg["RR"])[0]
                    rl_pos = p.getLinkState(robot, toe_link_by_leg["RL"])[0]
                    rear_mid_x = (rr_pos[0] + rl_pos[0]) * 0.5
                    rear_mid_y = (rr_pos[1] + rl_pos[1]) * 0.5
                    rear_min_z = min(rr_pos[2], rl_pos[2])
                    target_pos = [
                        target_pos[0] + rear_anchor[0] - rear_mid_x,
                        target_pos[1] + rear_anchor[1] - rear_mid_y,
                        target_pos[2] + 0.035 - rear_min_z,
                    ]
                    p.resetBasePositionAndOrientation(robot, target_pos, target_orn)

                p.resetBaseVelocity(robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                p.stepSimulation()
                time.sleep(dt)
                return True
            except p.error:
                return False

        settle_steps = max(1, int(0.26 / dt))
        for _ in range(settle_steps):
            if not step_beg(0.0, 0.0):
                return

        rise_steps = max(1, int(0.68 / dt))
        for i in range(rise_steps):
            phase = smoothstep01((i + 1) / float(rise_steps))
            if not step_beg(phase, 0.0):
                return

        hold_steps = max(1, int(1.20 / dt))
        for i in range(hold_steps):
            phase = (i + 1) / float(hold_steps)
            paw = math.sin(phase * math.pi * 5.0)
            if not step_beg(1.0, paw):
                return

        lower_steps = max(1, int(0.60 / dt))
        for i in range(lower_steps):
            phase = 1.0 - smoothstep01((i + 1) / float(lower_steps))
            if not step_beg(phase, 0.0):
                return

        settle_back_steps = max(1, int(0.32 / dt))
        for _ in range(settle_back_steps):
            if not step_beg(0.0, 0.0):
                return

    def execute_shake_hand_action():
        nonlocal vx, vy, wz, target_vx, target_vy, target_wz, posture_blend, gait_phase, external_posture
        nonlocal idle_hold_active
        if not body_exists(robot):
            return

        fr_joints = leg_joints.get("FR", {})
        if not all(k in fr_joints for k in ("hip", "upper", "lower")):
            return

        try:
            pos0, orn0 = p.getBasePositionAndOrientation(robot)
            _, _, yaw0 = p.getEulerFromQuaternion(orn0)
        except p.error:
            return

        vx = vy = wz = 0.0
        target_vx = target_vy = target_wz = 0.0
        gait_phase = 0.0
        posture_blend = 1.0
        external_posture = "stand"
        idle_hold_active = False

        def angle_delta(a, b):
            return math.atan2(math.sin(a - b), math.cos(a - b))

        def smoothstep01(t):
            t = clamp(t, 0.0, 1.0)
            return t * t * (3.0 - 2.0 * t)

        def build_shake_targets(lift, wave):
            targets = dict(stand_targets_by_id)

            for leg, hip_offset, upper_offset, lower_offset in (
                ("FL", 0.04, -0.04, 0.06),
                ("RR", 0.02, -0.03, 0.04),
                ("RL", 0.02, -0.03, 0.04),
            ):
                joints = leg_joints.get(leg, {})
                if not all(k in joints for k in ("hip", "upper", "lower")):
                    continue
                hip_id = joints["hip"]
                upper_id = joints["upper"]
                lower_id = joints["lower"]
                targets[hip_id] = clamp_joint(
                    joint_name_by_id[hip_id],
                    stand_targets_by_id[hip_id] + hip_offset * lift,
                )
                targets[upper_id] = clamp_joint(
                    joint_name_by_id[upper_id],
                    stand_targets_by_id[upper_id] + upper_offset * lift,
                )
                targets[lower_id] = clamp_joint(
                    joint_name_by_id[lower_id],
                    stand_targets_by_id[lower_id] + lower_offset * lift,
                )

            hip_id = fr_joints["hip"]
            upper_id = fr_joints["upper"]
            lower_id = fr_joints["lower"]
            hip = stand_targets_by_id[hip_id] + 0.16 * lift + 0.05 * wave
            upper = stand_targets_by_id[upper_id] + 0.48 * lift + 0.08 * wave
            lower = stand_targets_by_id[lower_id] - 0.94 * lift - 0.14 * wave
            targets[hip_id] = clamp_joint(joint_name_by_id[hip_id], hip)
            targets[upper_id] = clamp_joint(joint_name_by_id[upper_id], upper)
            targets[lower_id] = clamp_joint(joint_name_by_id[lower_id], lower)
            return targets

        def step_handshake(lift, wave):
            if not body_exists(robot):
                return False
            try:
                pos, orn = p.getBasePositionAndOrientation(robot)
                roll, pitch, yaw = p.getEulerFromQuaternion(orn)
                lin_vel, ang_vel = p.getBaseVelocity(robot)
            except p.error:
                return False

            x_err = pos[0] - pos0[0]
            y_err = pos[1] - pos0[1]
            yaw_err = angle_delta(yaw0, yaw)
            support_height = 0.43 + 0.02 * lift
            fx = clamp(-168.0 * x_err - 38.0 * lin_vel[0], -72.0, 72.0)
            fy = clamp(-182.0 * y_err - 38.0 * lin_vel[1], -84.0, 84.0)
            fz = clamp(44.0 * (support_height - pos[2]) - 52.0 * lin_vel[2], -66.0, 42.0)
            tx = clamp(-190.0 * roll - 34.0 * ang_vel[0], -58.0, 58.0)
            ty = clamp(-178.0 * pitch - 32.0 * ang_vel[1], -54.0, 54.0)
            tz = clamp(124.0 * yaw_err - 24.0 * ang_vel[2], -42.0, 42.0)

            try:
                apply_joint_targets(robot, build_shake_targets(lift, wave))
                p.applyExternalForce(robot, -1, [fx, fy, fz], [0.0, 0.0, 0.0], p.WORLD_FRAME)
                p.applyExternalTorque(robot, -1, [tx, ty, tz], p.WORLD_FRAME)
                p.stepSimulation()
                time.sleep(dt)
                return True
            except p.error:
                return False

        settle_steps = max(1, int(0.30 / dt))
        for _ in range(settle_steps):
            if not step_handshake(0.0, 0.0):
                return

        raise_steps = max(1, int(0.42 / dt))
        for i in range(raise_steps):
            phase = smoothstep01((i + 1) / float(raise_steps))
            if not step_handshake(phase, 0.0):
                return

        wave_steps = max(1, int(1.25 / dt))
        for i in range(wave_steps):
            phase = (i + 1) / float(wave_steps)
            wave = math.sin(phase * math.pi * 6.0)
            if not step_handshake(1.0, wave):
                return

        lower_steps = max(1, int(0.40 / dt))
        for i in range(lower_steps):
            phase = 1.0 - smoothstep01((i + 1) / float(lower_steps))
            if not step_handshake(phase, 0.0):
                return

        settle_back_steps = max(1, int(0.28 / dt))
        for _ in range(settle_back_steps):
            if not step_handshake(0.0, 0.0):
                return

    def apply_base_cmd(vx_cmd, vy_cmd, wz_cmd):
        nonlocal gait_phase, external_posture, posture_blend
        nonlocal idle_anchor_x, idle_anchor_y, idle_anchor_yaw, idle_hold_active
        if not body_exists(robot):
            return False
        try:
            pos, orn = p.getBasePositionAndOrientation(robot)
            roll, pitch, yaw = p.getEulerFromQuaternion(orn)
            lin_vel, ang_vel = p.getBaseVelocity(robot)

            if max(abs(vx_cmd), abs(vy_cmd), abs(wz_cmd)) >= 0.03:
                idle_hold_active = False

            desired_posture_blend = 0.0 if (external_cmd_file and external_posture == "sit") else 1.0
            blend_step = max(0.0, posture_blend_rate) * dt
            if posture_blend < desired_posture_blend:
                posture_blend = min(desired_posture_blend, posture_blend + blend_step)
            elif posture_blend > desired_posture_blend:
                posture_blend = max(desired_posture_blend, posture_blend - blend_step)

            if external_cmd_file and external_posture == "sit":
                # Let robot decelerate first, then settle into sit smoothly.
                planar_speed = math.hypot(lin_vel[0], lin_vel[1])
                yaw_rate = abs(ang_vel[2])
                cmd_level_for_sit = max(abs(vx_cmd), abs(vy_cmd), abs(wz_cmd))
                if (
                    planar_speed < sit_speed_threshold
                    and yaw_rate < sit_turn_threshold
                    and cmd_level_for_sit < 0.03
                ):
                    gait_phase = 0.0
                    if posture_blend <= 0.02:
                        hold_sit_pose(robot)
                    else:
                        apply_joint_targets(robot, blend_static_pose(posture_blend))
                    return True

            # Only recover if it fully tips; do not overwrite pose every frame.
            if abs(roll) > 1.15 or abs(pitch) > 1.15 or pos[2] < 0.10:
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
                if external_cmd_file and posture_blend < 0.995:
                    apply_joint_targets(robot, blend_static_pose(posture_blend))
                else:
                    hold_stand_pose(robot)
                if not idle_hold_active:
                    idle_anchor_x = pos[0]
                    idle_anchor_y = pos[1]
                    idle_anchor_yaw = yaw
                    idle_hold_active = True
                x_err = pos[0] - idle_anchor_x
                y_err = pos[1] - idle_anchor_y
                yaw_err = math.atan2(math.sin(idle_anchor_yaw - yaw), math.cos(idle_anchor_yaw - yaw))
                fx = clamp(-155.0 * x_err - 36.0 * lin_vel[0], -82.0, 82.0)
                fy = clamp(-155.0 * y_err - 36.0 * lin_vel[1], -82.0, 82.0)
                fz = clamp(36.0 * (0.41 - pos[2]) - 40.0 * lin_vel[2], -55.0, 30.0)
                tx = clamp(-172.0 * roll - 32.0 * ang_vel[0], -52.0, 52.0)
                ty = clamp(-188.0 * pitch - 34.0 * ang_vel[1], -58.0, 58.0)
                tz = clamp(120.0 * yaw_err - 24.0 * ang_vel[2], -44.0, 44.0)
                p.applyExternalForce(robot, -1, [fx, fy, fz], [0.0, 0.0, 0.0], p.WORLD_FRAME)
                p.applyExternalTorque(robot, -1, [tx, ty, tz], p.WORLD_FRAME)
                if abs(roll) > 1.05 or abs(pitch) > 1.05 or pos[2] < 0.12:
                    p.resetBasePositionAndOrientation(
                        robot,
                        [idle_anchor_x, idle_anchor_y, 0.46],
                        p.getQuaternionFromEuler([0.0, 0.0, idle_anchor_yaw]),
                    )
                    p.resetBaseVelocity(robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                    hold_stand_pose(robot)
                return True
            idle_hold_active = False

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
                strafe_bias = clamp(abs(body_lat) - 0.35 * abs(body_fwd) - 0.25 * abs(wz_eff), 0.0, 1.0)
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                target_lin_x = (body_fwd * cos_yaw - body_lat * sin_yaw) * max_v
                target_lin_y = (body_fwd * sin_yaw + body_lat * cos_yaw) * max_v
                target_yaw_rate = wz_eff * max_wz
                if strafe_bias > 0.05 and abs(wz_eff) < 0.10:
                    yaw_err = math.atan2(math.sin(-yaw), math.cos(-yaw))
                    target_yaw_rate += strafe_bias * clamp(2.2 * yaw_err - 0.8 * ang_vel[2], -0.55, 0.55)
                force_x = clamp(
                    185.0 * (target_lin_x - lin_vel[0]) + 18.0 * body_fwd - 115.0 * strafe_bias * lin_vel[0],
                    -62.0,
                    62.0,
                )
                force_y = clamp(175.0 * (target_lin_y - lin_vel[1]) + 16.0 * body_lat, -52.0, 52.0)
                p.applyExternalForce(
                    robot,
                    -1,
                    [force_x, force_y, 0.0],
                    [0.0, 0.0, 0.0],
                    p.WORLD_FRAME,
                )
                roll_torque = clamp(-90.0 * roll - 16.0 * ang_vel[0], -24.0, 24.0)
                pitch_torque = clamp(-105.0 * pitch - 18.0 * ang_vel[1], -28.0, 28.0)
                yaw_torque = clamp(42.0 * (target_yaw_rate - ang_vel[2]) + 8.0 * wz_eff, -24.0, 24.0)
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
                walk_phi = phi
                phase01 = (walk_phi / (2.0 * math.pi)) % 1.0
                swing = math.sin(walk_phi)              # fore/aft phase
                lift = max(0.0, math.sin(walk_phi))     # swing-leg lift [0..1]
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
                    # Keep the flat-ground fore/aft step mild so the external
                    # translation controller sets direction instead of the legs
                    # fighting positive vx on every other half-cycle.
                    sweep = -0.05 * (flat_body_forward_sign * vx_eff) + 0.12 * wz_eff * yaw_leg
                else:
                    sweep = 0.22 * vx_eff + 0.16 * wz_eff * yaw_leg
                if climbing_active:
                    sweep *= 1.55

                if flat_mode:
                    hip = stand_targets_by_id[hip_id] + side * (0.075 * vy_eff + 0.035 * wz_eff + 0.010 * vy_eff * swing) + side * (0.006 * swing)
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

    if external_cmd_file:
        print("[quad_sim] commands come from main.py (follow/patrol/greet/beg/shake_hand/sit/stand/stop)")
    else:
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

            external_cmd = read_external_command()
            if external_cmd is not None:
                target_vx = external_cmd["vx"]
                target_vy = external_cmd["vy"]
                target_wz = external_cmd["wz"]
                external_posture = external_cmd["posture"]
                external_action = external_cmd["action"]
                external_action_id = external_cmd["action_id"]
                if external_posture == "sit":
                    target_vx = target_vy = target_wz = 0.0
                if external_action == "beg" and external_action_id > last_external_action_id:
                    last_external_action_id = external_action_id
                    execute_beg_action()
                    target_vx = target_vy = target_wz = 0.0
                if external_action == "greet" and external_action_id > last_external_action_id:
                    last_external_action_id = external_action_id
                    execute_greet_action()
                    target_vx = target_vy = target_wz = 0.0
                if external_action == "shake_hand" and external_action_id > last_external_action_id:
                    last_external_action_id = external_action_id
                    execute_shake_hand_action()
                    target_vx = target_vy = target_wz = 0.0
                w_down = s_down = q_down = e_down = a_down = d_down = False

            target_vx = clamp(target_vx, -max_v, max_v)
            target_vy = clamp(target_vy, -max_v, max_v)
            target_wz = clamp(target_wz, -max_wz, max_wz)

            using_external = external_cmd is not None
            if using_external:
                lin_gain = external_lin_gain
                wz_gain = external_wz_gain
                lin_accel = external_lin_accel
                wz_accel = external_wz_accel
            else:
                lin_gain = manual_gain
                wz_gain = manual_gain
                lin_accel = manual_lin_accel
                wz_accel = manual_wz_accel

            vx = smooth_axis(vx, target_vx, lin_gain, lin_accel)
            vy = smooth_axis(vy, target_vy, lin_gain, lin_accel)
            wz = smooth_axis(wz, target_wz, wz_gain, wz_accel)

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
