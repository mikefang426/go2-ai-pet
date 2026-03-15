from dataclasses import dataclass
from typing import Optional

try:
    from unitree_sdk2py.go2.sport.sport_client import SportClient
except ModuleNotFoundError as exc:
    SportClient = None
    _UNITREE_IMPORT_ERROR = exc

from .movement import MotionCommand, clamp_velocity


@dataclass
class Go2ControllerConfig:
    network_interface: Optional[str] = None
    max_v: float = 0.5
    max_wz: float = 1.0


class Go2Controller:
    """Thin wrapper around Unitree SportClient."""

    def __init__(self, config: Go2ControllerConfig | None = None) -> None:
        if SportClient is None:
            raise ModuleNotFoundError(
                "unitree_sdk2py is not installed. For real robot control, run: "
                "`pip install -r requirements-robot.txt` "
                "on Python 3.11/3.12, and configure CycloneDDS "
                "(CYCLONEDDS_HOME/CMAKE_PREFIX_PATH)."
            ) from _UNITREE_IMPORT_ERROR
        self.config = config or Go2ControllerConfig()
        self.client = SportClient()
        self._connected = False

    def connect(self) -> None:
        # 0 means default timeout in SDK examples.
        self.client.SetTimeout(10.0)
        self.client.Init()
        self._connected = True

    def ensure_connected(self) -> None:
        if not self._connected:
            self.connect()

    def stand(self) -> None:
        self.ensure_connected()
        self.client.StandUp()

    def sit(self) -> None:
        self.ensure_connected()
        self.client.StandDown()

    def stop(self) -> None:
        self.move(MotionCommand())

    def move(self, cmd: MotionCommand) -> None:
        self.ensure_connected()
        safe = clamp_velocity(cmd, max_v=self.config.max_v, max_wz=self.config.max_wz)
        self.client.Move(safe.vx, safe.vy, safe.wz)
