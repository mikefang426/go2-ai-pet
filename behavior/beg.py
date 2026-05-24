from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


class Beg:
    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        if context and context.interrupted:
            controller.stop()
            return

        beg_fn = getattr(controller, "beg", None)
        if callable(beg_fn):
            beg_fn()
            return

        print("Beg is only supported in simulator mode for safety.")
