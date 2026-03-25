from interfaces.robot_interface import RobotInterface


class Flip:
    def run(self, controller: RobotInterface) -> None:
        flip_fn = getattr(controller, "flip", None)
        if callable(flip_fn):
            flip_fn()
            return
        print("Flip is only supported in simulator mode for safety.")
