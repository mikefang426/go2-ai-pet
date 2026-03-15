from dataclasses import dataclass


@dataclass
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def clamp_velocity(cmd: MotionCommand, max_v: float = 0.5, max_wz: float = 1.0) -> MotionCommand:
    cmd.vx = max(-max_v, min(max_v, cmd.vx))
    cmd.vy = max(-max_v, min(max_v, cmd.vy))
    cmd.wz = max(-max_wz, min(max_wz, cmd.wz))
    return cmd
