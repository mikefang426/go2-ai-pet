from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


class GreetPerson:
    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        if context and context.interrupted:
            controller.stop()
            return

        greet_fn = getattr(controller, "greet", None)
        if callable(greet_fn):
            greet_fn()
            return

        controller.stop()
