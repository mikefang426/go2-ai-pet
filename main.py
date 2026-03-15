from ai.llm_agent import LLMAgent
from behavior.follow_user import FollowUser
from behavior.greet_person import GreetPerson
from behavior.patrol import Patrol
from robot.go2_controller import Go2Controller


def main() -> None:
    controller = Go2Controller()
    agent = LLMAgent()

    behaviors = {
        "greet_person": GreetPerson(),
        "follow_user": FollowUser(),
        "patrol": Patrol(),
    }

    print("GO2 AI Pet started. 输入指令，例如: 跟随 / 巡逻 / 打招呼 / 停止")
    while True:
        user_text = input("> ").strip()
        if user_text in {"exit", "quit", "退出"}:
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
