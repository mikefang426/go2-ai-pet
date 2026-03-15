import argparse

from ai.llm_agent import LLMAgent
from behavior.follow_user import FollowUser
from behavior.greet_person import GreetPerson
from behavior.patrol import Patrol
from interfaces.robot_interface import RobotInterface


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

    behaviors = {
        "greet_person": GreetPerson(),
        "follow_user": FollowUser(),
        "patrol": Patrol(),
    }

    print(
        f"GO2 AI Pet started in {args.mode} mode. "
        "Enter a command, e.g.: follow / patrol / greet / sit / stand / stop"
    )
    while True:
        user_text = input("> ").strip()
        if user_text in {"exit", "quit"}:
            controller.stop()
            break

        intent = agent.infer_intent(user_text)
        if intent.name == "stop":
            controller.stop()
            continue
        if intent.name == "sit":
            controller.sit()
            continue
        if intent.name == "stand":
            controller.stand()
            continue

        behavior = behaviors.get(intent.name)
        if behavior is None:
            print(f"Unknown intent: {intent.name}")
            continue
        behavior.run(controller)


if __name__ == "__main__":
    main()
