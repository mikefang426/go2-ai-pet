from enum import Enum


class RobotAction(str, Enum):
    IDLE = "idle"
    BEG = "beg"
    GREET = "greet"
    FOLLOW = "follow"
    PATROL = "patrol"
    SHAKE_HAND = "shake_hand"
    SIT = "sit"
    STAND = "stand"
