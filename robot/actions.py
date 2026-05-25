from enum import Enum


class RobotAction(str, Enum):
    IDLE = "idle"
    GREET = "greet"
    FOLLOW = "follow"
    PATROL = "patrol"
    SIT = "sit"
    STAND = "stand"
