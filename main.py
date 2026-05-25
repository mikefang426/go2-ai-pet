import argparse

from ai.llm_agent import LLMAgent
from behavior.follow_user import FollowUser
from behavior.greet_person import GreetPerson
from behavior.patrol import Patrol
from behavior.sit import Sit
from behavior.stand import Stand
from interfaces.robot_interface import RobotInterface
from planner.behavior_runner import BehaviorRunner


def build_robot(mode: str) -> RobotInterface:
    if mode == "sim":
        from simulation.sim_controller import SimRobot as Robot
    else:
        from robot.go2_controller import Go2Robot as Robot

    return Robot()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    args = parser.parse_args()

    controller = build_robot(args.mode)
    agent = LLMAgent()
    runner = BehaviorRunner(controller)

    behaviors = {
        "greet_person": GreetPerson(),
        "follow_user": FollowUser(),
        "patrol": Patrol(),
        "sit": Sit(),
        "stand": Stand(),
    }

    print(
        f"GO2 AI Pet started in {args.mode} mode. "
        "Enter a command, e.g.: follow / patrol / greet / sit / stand / stop"
    )

    def shutdown() -> None:
        runner.stop()
        controller.stop()

    try:
        while True:
            user_text = input("> ").strip()
            if not user_text:
                continue
            if user_text in {"exit", "quit"}:
                shutdown()
                break

            intent = agent.infer_intent(user_text)
            if intent.name == "stop":
                shutdown()
                continue

            behavior = behaviors.get(intent.name)
            if behavior is None:
                print(f"Unknown intent: {intent.name}")
                continue
            if not runner.start(intent.name, behavior):
                print(f"Could not start {intent.name}: active behavior did not stop in time.")
    except (KeyboardInterrupt, EOFError):
        print()
        shutdown()
    finally:
        if runner.is_running:
            runner.stop()
            controller.stop()


if __name__ == "__main__":
    main()
